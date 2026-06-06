## V2: 16x16 IMG

"""
=============================================================================
Neural ODE Classifier (16×16) — Verification-Friendly Version for TNODEV
=============================================================================
Run:  python pyopia_neural_ode_16x16_verification.py

Architecture:
  Pre-ODE:  Conv2d(3->4) -> BN -> ReLU -> Conv2d(4->4,s=2) -> BN -> ReLU -> Flatten
            Input: 16x16x3  ->  Output: 8x8x4 = 256 dimensions

  ODE:      dx/dt = W2 * ReLU(W1*x + b1) + b2    (FC bottleneck)
            W1: 16x256,  W2: 256x16
            Maps to System_description.m case 19

  Post-ODE: FC(256->64) -> ReLU -> FC(64->7)

ODE state dimension: 256 (vs 32 for the 8x8 model, 676 for MNIST)
NOTE: 256-dim ODE state is intractable for TIRA CTMM verification.
      This model is trained for accuracy comparison only.
=============================================================================
"""

import os
import pathlib
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd

import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
from torchvision import transforms
from sklearn.model_selection import train_test_split
import sklearn.metrics
import scipy.io as sio

from torchdiffeq import odeint, odeint_adjoint

SEED = 42
np.random.seed(SEED)
torch.manual_seed(SEED)
torch.cuda.manual_seed_all(SEED)

DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"Using device: {DEVICE}")
if DEVICE.type == "cuda":
    print(f"GPU: {torch.cuda.get_device_name(0)}")


# ═══════════════════════════════════════════════════════════════════════════
#  Configuration — 16x16 verification model
# ═══════════════════════════════════════════════════════════════════════════
IMG_SIZE = 16           # larger input for better accuracy
N_FILTERS = 4           # 4 filters → state_dim = 4 × 8 × 8 = 256
BOTTLENECK_DIM = 16     # FC bottleneck in ODE dynamics
NUM_EPOCHS = 300        # more epochs since model is small
LEARNING_RATE = 0.01
BATCH_SIZE = 128
ODE_TOL = 1e-3
USE_ADJOINT = True
SOLVER = "dopri5"

# Derived
SPATIAL_AFTER_DOWNSAMPLE = IMG_SIZE // 2  # 8
ODE_STATE_DIM = N_FILTERS * SPATIAL_AFTER_DOWNSAMPLE * SPATIAL_AFTER_DOWNSAMPLE  # 256

print(f"\n--- Verification Model Config (16x16) ---")
print(f"Image size:     {IMG_SIZE}x{IMG_SIZE}")
print(f"Filters:        {N_FILTERS}")
print(f"ODE state dim:  {ODE_STATE_DIM}")
print(f"Bottleneck dim: {BOTTLENECK_DIM}")
print(f"ODE dynamics:   dx/dt = W2 * ReLU(W1*x + b1) + b2")
print(f"                W1: {BOTTLENECK_DIM}x{ODE_STATE_DIM}")
print(f"                W2: {ODE_STATE_DIM}x{BOTTLENECK_DIM}")


# ═══════════════════════════════════════════════════════════════════════════
#  1. Download database
# ═══════════════════════════════════════════════════════════════════════════
print("\n--- Downloading PyOPIA particle database ---")
import pyopia.exampledata

db_path = pyopia.exampledata.get_classifier_database_from_pysilcam_blob(
    download_directory="silcam_classification_database"
)

CLASS_LABELS = ["bubble", "copepod", "diatom_chain",
                "faecal_pellets", "oil", "oily_gas", "other"]
NUM_CLASSES = len(CLASS_LABELS)
COPEPOD_IDX = CLASS_LABELS.index("copepod")

all_image_paths = []
all_labels = []
for idx, cl in enumerate(CLASS_LABELS):
    paths = sorted(list(pathlib.Path(db_path, cl).glob("*.tiff")))
    all_image_paths.extend(paths)
    all_labels.extend([idx] * len(paths))
    print(f"  {cl}: {len(paths)} images")
print(f"Total: {len(all_image_paths)}")


# ═══════════════════════════════════════════════════════════════════════════
#  2. Dataset
# ═══════════════════════════════════════════════════════════════════════════
train_transform = transforms.Compose([
    transforms.ToPILImage(),
    transforms.Resize((IMG_SIZE, IMG_SIZE)),
    transforms.RandomHorizontalFlip(),
    transforms.RandomRotation(15),
    transforms.ColorJitter(brightness=0.2, contrast=0.2),
    transforms.ToTensor(),
])

val_transform = transforms.Compose([
    transforms.ToPILImage(),
    transforms.Resize((IMG_SIZE, IMG_SIZE)),
    transforms.ToTensor(),
])


class ParticleDataset(Dataset):
    def __init__(self, image_paths, labels, transform=None):
        self.image_paths = image_paths
        self.labels = labels
        self.transform = transform

    def __len__(self):
        return len(self.image_paths)

    def __getitem__(self, idx):
        img = plt.imread(str(self.image_paths[idx]))
        if img.dtype in (np.float32, np.float64):
            img = (img * 255).clip(0, 255).astype(np.uint8)
        if img.ndim == 2:
            img = np.stack([img] * 3, axis=-1)
        elif img.shape[-1] == 4:
            img = img[..., :3]
        if self.transform:
            img = self.transform(img)
        else:
            img = transforms.ToTensor()(img)
        return img, self.labels[idx]


train_paths, val_paths, train_labels, val_labels = train_test_split(
    all_image_paths, all_labels, test_size=0.2,
    stratify=all_labels, random_state=SEED
)

train_ds = ParticleDataset(train_paths, train_labels, transform=train_transform)
val_ds = ParticleDataset(val_paths, val_labels, transform=val_transform)

train_loader = DataLoader(train_ds, batch_size=BATCH_SIZE, shuffle=True,
                          num_workers=2, pin_memory=True, drop_last=True)
val_loader = DataLoader(val_ds, batch_size=BATCH_SIZE, shuffle=False,
                        num_workers=2, pin_memory=True)

print(f"Train: {len(train_ds)}  |  Val: {len(val_ds)}")


# ═══════════════════════════════════════════════════════════════════════════
#  3. Model — Verification-friendly architecture
# ═══════════════════════════════════════════════════════════════════════════

class ODEFuncFC(nn.Module):
    """FC-based ODE dynamics: dx/dt = W2 * ReLU(W1*x + b1) + b2

    Same structure as MNIST example in NNV.
    Maps to System_description.m case 19.
    """

    def __init__(self, state_dim, bottleneck_dim):
        super().__init__()
        self._num_evals = 0
        self.fc1 = nn.Linear(state_dim, bottleneck_dim)
        self.fc2 = nn.Linear(bottleneck_dim, state_dim)

    def forward(self, t, x):
        self._num_evals += 1
        h = F.relu(self.fc1(x))
        return self.fc2(h)


class ODEBlock(nn.Module):
    def __init__(self, odefunc, solver="dopri5", rtol=1e-3, atol=1e-3,
                 use_adjoint=False):
        super().__init__()
        self.odefunc = odefunc
        self.solver = solver
        self.rtol = rtol
        self.atol = atol
        self.use_adjoint = use_adjoint
        self.register_buffer(
            "integration_time",
            torch.tensor([0.0, 1.0], dtype=torch.float32),
        )

    def forward(self, x):
        solve_fn = odeint_adjoint if self.use_adjoint else odeint
        out = solve_fn(
            self.odefunc, x, self.integration_time,
            method=self.solver, rtol=self.rtol, atol=self.atol,
        )
        return out[-1]


class TinyODEnet(nn.Module):
    """Neural ODE for verification.

    Structure:
      Pre-ODE:  Conv -> BN -> ReLU -> Conv(s=2) -> BN -> ReLU -> Flatten
      ODE:      FC bottleneck dynamics (case 19 in System_description.m)
      Post-ODE: FC -> ReLU -> FC
    """

    def __init__(self, in_channels=3, n_filters=4, state_dim=256,
                 bottleneck_dim=16, num_classes=7, solver="dopri5",
                 tol=1e-3, use_adjoint=False):
        super().__init__()
        self.state_dim = state_dim

        self.pre_conv1 = nn.Conv2d(in_channels, n_filters, 3, stride=1,
                                   padding=1, bias=True)
        self.pre_bn1 = nn.BatchNorm2d(n_filters)

        self.pre_conv2 = nn.Conv2d(n_filters, n_filters, 3, stride=2,
                                   padding=1, bias=True)
        self.pre_bn2 = nn.BatchNorm2d(n_filters)

        self.ode_block = ODEBlock(
            odefunc=ODEFuncFC(state_dim, bottleneck_dim),
            solver=solver, rtol=tol, atol=tol,
            use_adjoint=use_adjoint,
        )

        self.post_fc1 = nn.Linear(state_dim, 64)
        self.post_fc2 = nn.Linear(64, num_classes)

    def forward(self, x):
        # Pre-ODE
        x = F.relu(self.pre_bn1(self.pre_conv1(x)))   # [B, 4, 16, 16]
        x = F.relu(self.pre_bn2(self.pre_conv2(x)))   # [B, 4, 8, 8]
        x = x.flatten(1)                               # [B, 256]

        # ODE
        x = self.ode_block(x)                           # [B, 256]

        # Post-ODE
        x = F.relu(self.post_fc1(x))                   # [B, 64]
        x = self.post_fc2(x)                            # [B, 7]
        return x

    def forward_with_intermediates(self, x):
        """Forward pass returning intermediate outputs for debugging
        PyTorch vs MATLAB mismatch."""
        # Pre-ODE
        x = F.relu(self.pre_bn1(self.pre_conv1(x)))
        x = F.relu(self.pre_bn2(self.pre_conv2(x)))
        pre_ode_4d = x.clone()           # [B, C, H, W] before flatten
        x = x.flatten(1)
        pre_ode_flat = x.clone()          # [B, 256] after flatten (PyTorch C-first order)

        # ODE
        x = self.ode_block(x)
        post_ode = x.clone()             # [B, 256] after ODE integration

        # Post-ODE
        x = F.relu(self.post_fc1(x))
        post_fc1_out = x.clone()         # [B, 64] after FC1+ReLU
        x = self.post_fc2(x)
        logits = x.clone()               # [B, 7] final output

        return {
            "pre_ode_4d": pre_ode_4d,
            "pre_ode_flat": pre_ode_flat,
            "post_ode": post_ode,
            "post_fc1": post_fc1_out,
            "logits": logits,
        }


def count_parameters(model):
    return sum(p.numel() for p in model.parameters() if p.requires_grad)


# ═══════════════════════════════════════════════════════════════════════════
#  4. Training functions
# ═══════════════════════════════════════════════════════════════════════════

def train_one_epoch(model, loader, criterion, optimizer, scheduler=None):
    model.train()
    running_loss = 0.0
    correct = 0
    total = 0
    for images, labels in loader:
        images, labels = images.to(DEVICE), labels.to(DEVICE)
        optimizer.zero_grad()
        outputs = model(images)
        loss = criterion(outputs, labels)
        loss.backward()
        torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
        optimizer.step()
        running_loss += loss.item() * images.size(0)
        _, predicted = outputs.max(1)
        total += labels.size(0)
        correct += predicted.eq(labels).sum().item()
    if scheduler is not None:
        scheduler.step()
    return running_loss / total, 100.0 * correct / total


@torch.no_grad()
def evaluate(model, loader, criterion):
    model.eval()
    running_loss = 0.0
    correct = 0
    total = 0
    all_preds = []
    all_labels = []
    for images, labels in loader:
        images, labels = images.to(DEVICE), labels.to(DEVICE)
        outputs = model(images)
        loss = criterion(outputs, labels)
        running_loss += loss.item() * images.size(0)
        _, predicted = outputs.max(1)
        total += labels.size(0)
        correct += predicted.eq(labels).sum().item()
        all_preds.extend(predicted.cpu().numpy())
        all_labels.extend(labels.cpu().numpy())
    return (running_loss / total, 100.0 * correct / total,
            np.array(all_preds), np.array(all_labels))


def copepod_recall(y_true, y_pred, idx=1):
    mask = (y_true == idx)
    if mask.sum() == 0:
        return 0.0
    return 100.0 * (y_pred[mask] == idx).sum() / mask.sum()


# ═══════════════════════════════════════════════════════════════════════════
#  5. Train
# ═══════════════════════════════════════════════════════════════════════════
print("\n--- Building model ---")

model = TinyODEnet(
    in_channels=3,
    n_filters=N_FILTERS,
    state_dim=ODE_STATE_DIM,
    bottleneck_dim=BOTTLENECK_DIM,
    num_classes=NUM_CLASSES,
    solver=SOLVER,
    tol=ODE_TOL,
    use_adjoint=USE_ADJOINT,
).to(DEVICE)

print(f"Parameters: {count_parameters(model):,}")
print(f"\nModel structure:")
print(model)

# Mild copepod weight
loss_weights = torch.ones(NUM_CLASSES)
loss_weights[COPEPOD_IDX] = 1.5
criterion = nn.CrossEntropyLoss(weight=loss_weights.to(DEVICE), label_smoothing=0.1)

optimizer = optim.SGD(model.parameters(), lr=LEARNING_RATE,
                      momentum=0.9, weight_decay=1e-4)
scheduler = optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=NUM_EPOCHS)

print(f"\n--- Training ---")
print(f"Epochs: {NUM_EPOCHS} | LR: {LEARNING_RATE} | Batch: {BATCH_SIZE}")

history = {"train_loss": [], "train_acc": [], "val_loss": [], "val_acc": [],
           "copepod_recall": []}
best_val_acc = 0.0
best_copepod_recall = 0.0

for epoch in range(1, NUM_EPOCHS + 1):
    model.ode_block.odefunc._num_evals = 0

    train_loss, train_acc = train_one_epoch(
        model, train_loader, criterion, optimizer, scheduler
    )
    val_loss, val_acc, y_pred, y_true = evaluate(model, val_loader, criterion)
    cop_recall = copepod_recall(y_true, y_pred, COPEPOD_IDX)
    nfe = model.ode_block.odefunc._num_evals

    history["train_loss"].append(train_loss)
    history["train_acc"].append(train_acc)
    history["val_loss"].append(val_loss)
    history["val_acc"].append(val_acc)
    history["copepod_recall"].append(cop_recall)

    if val_acc > best_val_acc:
        best_val_acc = val_acc
        best_copepod_recall = cop_recall
        torch.save(model.state_dict(), "best_odenet_16x16.pth")

    if epoch % 20 == 0 or epoch == 1:
        print(f"Epoch {epoch:3d}/{NUM_EPOCHS}  "
              f"Train: {train_loss:.4f} / {train_acc:.1f}%  |  "
              f"Val: {val_loss:.4f} / {val_acc:.1f}%  |  "
              f"Copepod: {cop_recall:.1f}%  |  "
              f"NFE: {nfe}  |  Best: {best_val_acc:.1f}%")

    if epoch % 50 == 0:
        torch.save(model.state_dict(), f"checkpoint_16x16_epoch_{epoch}.pth")

print(f"\nTraining complete!")
print(f"Best overall accuracy: {best_val_acc:.2f}%")
print(f"Copepod recall at best: {best_copepod_recall:.2f}%")


# ═══════════════════════════════════════════════════════════════════════════
#  6. Evaluation
# ═══════════════════════════════════════════════════════════════════════════
print("\n--- Loading best model and evaluating ---")
model.load_state_dict(torch.load("best_odenet_16x16.pth", map_location=DEVICE))
val_loss, val_acc, y_pred, y_true = evaluate(model, val_loader, criterion)

# Confusion matrix
cm = 100 * sklearn.metrics.confusion_matrix(y_true, y_pred, normalize="true")
cm_df = pd.DataFrame(cm, index=CLASS_LABELS, columns=CLASS_LABELS)

plt.figure(figsize=(9, 8))
sns.heatmap(cm_df, annot=True, fmt=".1f", vmin=0, vmax=100, cmap="Blues")
plt.xlabel("Predicted label")
plt.ylabel("True label")
plt.title(f"ODEnet 16x16 (verification) - Val Accuracy: {val_acc:.2f}%")
plt.tight_layout()
plt.savefig("confusion_matrix_16x16.png", dpi=150)
plt.show()

print("\n--- Classification Report ---")
print(sklearn.metrics.classification_report(
    y_true, y_pred, target_names=CLASS_LABELS
))

# Copepod detail
cop_mask = (y_true == COPEPOD_IDX)
cop_preds = y_pred[cop_mask]
print(f"--- Copepod detail ---")
print(f"Total copepod samples: {cop_mask.sum()}")
print(f"Correctly classified:  {(cop_preds == COPEPOD_IDX).sum()}")
print(f"Recall: {100 * (cop_preds == COPEPOD_IDX).sum() / cop_mask.sum():.1f}%")
print(f"Misclassified as:")
for i, cl in enumerate(CLASS_LABELS):
    if i != COPEPOD_IDX:
        n = (cop_preds == i).sum()
        if n > 0:
            print(f"  {cl}: {n} ({100*n/cop_mask.sum():.1f}%)")

# Training curves
fig, axes = plt.subplots(1, 3, figsize=(18, 5))
axes[0].plot(history["train_loss"], label="Train", linewidth=1.5)
axes[0].plot(history["val_loss"], label="Val", linewidth=1.5)
axes[0].set_xlabel("Epoch"); axes[0].set_ylabel("Loss")
axes[0].legend(); axes[0].set_title("Loss"); axes[0].grid(alpha=0.3)

axes[1].plot(history["train_acc"], label="Train", linewidth=1.5)
axes[1].plot(history["val_acc"], label="Val", linewidth=1.5)
axes[1].set_xlabel("Epoch"); axes[1].set_ylabel("Accuracy (%)")
axes[1].legend(); axes[1].set_title("Overall accuracy"); axes[1].grid(alpha=0.3)

axes[2].plot(history["copepod_recall"], label="Copepod recall",
             linewidth=2, color="#534AB7")
axes[2].set_xlabel("Epoch"); axes[2].set_ylabel("Recall (%)")
axes[2].legend(); axes[2].set_title("Copepod recall"); axes[2].grid(alpha=0.3)

plt.tight_layout()
plt.savefig("training_curves_16x16.png", dpi=150)
plt.show()


# ═══════════════════════════════════════════════════════════════════════════
#  7. Export weights to .mat for TNODEV
# ═══════════════════════════════════════════════════════════════════════════
print("\n--- Exporting to .mat for TNODEV ---")

model.eval()
model = model.double()  # Convert ALL weights to float64 for MATLAB
mat_data = {}

# Pre-ODE Conv1: Conv2d(3->4, 3x3, s=1, pad=1)
w = model.pre_conv1.weight.detach().cpu().numpy()   # [4, 3, 3, 3]
b = model.pre_conv1.bias.detach().cpu().numpy()      # [4]
mat_data["pre_conv1_W"] = np.transpose(w, (2, 3, 1, 0))  # -> [3, 3, 3, 4] MATLAB format
mat_data["pre_conv1_b"] = b
print(f"pre_conv1_W: {mat_data['pre_conv1_W'].shape}")

# Pre-ODE BN1: BatchNorm2d(4)
mat_data["pre_bn1_weight"] = model.pre_bn1.weight.detach().cpu().numpy()
mat_data["pre_bn1_bias"] = model.pre_bn1.bias.detach().cpu().numpy()
mat_data["pre_bn1_running_mean"] = model.pre_bn1.running_mean.detach().cpu().numpy()
mat_data["pre_bn1_running_var"] = model.pre_bn1.running_var.detach().cpu().numpy()
mat_data["pre_bn1_eps"] = model.pre_bn1.eps

# Pre-ODE Conv2: Conv2d(4->4, 3x3, s=2, pad=1)
w = model.pre_conv2.weight.detach().cpu().numpy()   # [4, 4, 3, 3]
b = model.pre_conv2.bias.detach().cpu().numpy()      # [4]
mat_data["pre_conv2_W"] = np.transpose(w, (2, 3, 1, 0))  # -> [3, 3, 4, 4]
mat_data["pre_conv2_b"] = b
print(f"pre_conv2_W: {mat_data['pre_conv2_W'].shape}")

# Pre-ODE BN2: BatchNorm2d(4)
mat_data["pre_bn2_weight"] = model.pre_bn2.weight.detach().cpu().numpy()
mat_data["pre_bn2_bias"] = model.pre_bn2.bias.detach().cpu().numpy()
mat_data["pre_bn2_running_mean"] = model.pre_bn2.running_mean.detach().cpu().numpy()
mat_data["pre_bn2_running_var"] = model.pre_bn2.running_var.detach().cpu().numpy()
mat_data["pre_bn2_eps"] = model.pre_bn2.eps

# ODE dynamics: W1, b1, W2, b2
# Maps to System_description.m case 19:
#   h = W1 * x + b1        (compress to bottleneck)
#   h_relu = max(0, h)      (ReLU on bottleneck neurons)
#   dx = W2 * h_relu + b2   (expand back to state dim)
ode_fc1_W = model.ode_block.odefunc.fc1.weight.detach().cpu().numpy()  # [16, 256]
ode_fc1_b = model.ode_block.odefunc.fc1.bias.detach().cpu().numpy()    # [16]
ode_fc2_W = model.ode_block.odefunc.fc2.weight.detach().cpu().numpy()  # [256, 16]
ode_fc2_b = model.ode_block.odefunc.fc2.bias.detach().cpu().numpy()    # [256]

mat_data["ode_W1"] = ode_fc1_W
mat_data["ode_b1"] = ode_fc1_b
mat_data["ode_W2"] = ode_fc2_W
mat_data["ode_b2"] = ode_fc2_b

print(f"ode_W1: {ode_fc1_W.shape}  (bottleneck x state)")
print(f"ode_b1: {ode_fc1_b.shape}")
print(f"ode_W2: {ode_fc2_W.shape}  (state x bottleneck)")
print(f"ode_b2: {ode_fc2_b.shape}")

# Post-ODE FC1: FC(256->64)
mat_data["post_fc1_W"] = model.post_fc1.weight.detach().cpu().numpy()  # [64, 256]
mat_data["post_fc1_b"] = model.post_fc1.bias.detach().cpu().numpy()    # [64]
print(f"post_fc1_W: {mat_data['post_fc1_W'].shape}")

# Post-ODE FC2: FC(64->7)
mat_data["post_fc2_W"] = model.post_fc2.weight.detach().cpu().numpy()  # [7, 64]
mat_data["post_fc2_b"] = model.post_fc2.bias.detach().cpu().numpy()    # [7]
print(f"post_fc2_W: {mat_data['post_fc2_W'].shape}")

# Metadata
mat_data["img_size"] = np.array([IMG_SIZE])
mat_data["n_filters"] = np.array([N_FILTERS])
mat_data["num_classes"] = np.array([NUM_CLASSES])
mat_data["ode_state_dim"] = np.array([ODE_STATE_DIM])
mat_data["bottleneck_dim"] = np.array([BOTTLENECK_DIM])
mat_data["ode_t_final"] = np.array([1.0])
mat_data["class_labels_str"] = ",".join(CLASS_LABELS)
mat_data["copepod_class_idx_matlab"] = np.array([COPEPOD_IDX + 1])  # 1-indexed

sio.savemat("odenet_16x16_copepod_weights.mat", mat_data)
print(f"\nSaved weights to: odenet_16x16_copepod_weights.mat")


# ═══════════════════════════════════════════════════════════════════════════
#  8. Export CORRECTLY CLASSIFIED copepod images WITH PyTorch intermediates
# ═══════════════════════════════════════════════════════════════════════════
print("\n--- Exporting verified copepod images with PyTorch intermediates ---")
model.eval()

correct_images = []          # [C, H, W] tensors
correct_logits = []          # final output logits [7]
correct_pre_ode_flat = []    # 256-dim vector after flatten (PyTorch C-first order)
correct_post_ode = []        # 256-dim vector after ODE integration

for i in range(len(val_paths)):
    if val_labels[i] != COPEPOD_IDX:
        continue

    img = plt.imread(str(val_paths[i]))
    if img.dtype in (np.float32, np.float64):
        img = (img * 255).clip(0, 255).astype(np.uint8)
    if img.ndim == 2:
        img = np.stack([img] * 3, axis=-1)
    elif img.shape[-1] == 4:
        img = img[..., :3]

    tensor = val_transform(img).unsqueeze(0).to(DEVICE).double()
    with torch.no_grad():
        intermediates = model.forward_with_intermediates(tensor)
        pred = intermediates["logits"].argmax(dim=1).item()

    if pred == COPEPOD_IDX:  # only export correctly classified images
        correct_images.append(tensor[0].cpu().numpy())                    # [C, H, W]
        correct_logits.append(intermediates["logits"][0].cpu().numpy())    # [7]
        correct_pre_ode_flat.append(intermediates["pre_ode_flat"][0].cpu().numpy())  # [256]
        correct_post_ode.append(intermediates["post_ode"][0].cpu().numpy())          # [256]

    if len(correct_images) >= 20:
        break

num_exported = len(correct_images)
images_array = np.stack(correct_images, axis=0)              # [N, C, H, W]
logits_array = np.stack(correct_logits, axis=0)              # [N, 7]
pre_ode_flat_array = np.stack(correct_pre_ode_flat, axis=0)  # [N, 256]
post_ode_array = np.stack(correct_post_ode, axis=0)          # [N, 256]

# Convert images to MATLAB format [H, W, C, N]
images_matlab = np.transpose(images_array, (2, 3, 1, 0))

sio.savemat("copepod_test_images_16x16.mat", {
    "images": images_matlab,                              # [H, W, C, N] for MATLAB
    "labels": np.ones(num_exported) * (COPEPOD_IDX + 1),  # 1-indexed for MATLAB
    "num_samples": np.array([num_exported]),
    "img_size": np.array([IMG_SIZE]),
    "class_name": "copepod",
    # PyTorch reference outputs for debugging MATLAB mismatch
    "pytorch_logits": logits_array,                # [N, 7]   — final class scores
    "pytorch_pre_ode_flat": pre_ode_flat_array,    # [N, 256] — after flatten (PyTorch order!)
    "pytorch_post_ode": post_ode_array,            # [N, 256] — after ODE integration
    "pytorch_pre_ode_4d_shape": np.array([N_FILTERS, SPATIAL_AFTER_DOWNSAMPLE,
                                           SPATIAL_AFTER_DOWNSAMPLE]),  # [C, H, W] = [4, 8, 8]
})

print(f"Exported {num_exported} correctly-classified copepod images")
print(f"PyTorch logits for image 1:       {logits_array[0]}")
print(f"PyTorch prediction for image 1:   {CLASS_LABELS[logits_array[0].argmax()]}")
print(f"PyTorch pre-ODE flat (first 8):   {pre_ode_flat_array[0][:8]}")
print(f"PyTorch post-ODE (first 8):       {post_ode_array[0][:8]}")


# ═══════════════════════════════════════════════════════════════════════════
#  9. Save full checkpoint
# ═══════════════════════════════════════════════════════════════════════════
torch.save({
    "model_state_dict": model.state_dict(),
    "class_labels": CLASS_LABELS,
    "img_size": IMG_SIZE,
    "n_filters": N_FILTERS,
    "state_dim": ODE_STATE_DIM,
    "bottleneck_dim": BOTTLENECK_DIM,
    "solver": SOLVER,
    "tol": ODE_TOL,
    "use_adjoint": USE_ADJOINT,
    "best_val_acc": best_val_acc,
    "best_copepod_recall": best_copepod_recall,
    "history": history,
}, "odenet_16x16_full.pth")


# ═══════════════════════════════════════════════════════════════════════════
#  10. Print summary
# ═══════════════════════════════════════════════════════════════════════════
print("\n" + "=" * 65)
print("  TNODEV MATLAB INTEGRATION (16x16 model)")
print("=" * 65)
print(f"""
# Files created:
#   1. odenet_16x16_copepod_weights.mat — network weights (float64)
#   2. copepod_test_images_16x16.mat    — {num_exported} verified copepod images
#      + pytorch_logits                 — PyTorch final output [N,7]
#      + pytorch_pre_ode_flat           — 256-dim flatten vector [N,256] (PyTorch order!)
#      + pytorch_post_ode               — 256-dim after ODE [N,256]

# NOTE: The 256-dim ODE state is intractable for TIRA CTMM verification.
#       This model is for accuracy comparison with the verified 8x8 model.

Model: {best_val_acc:.1f}% accuracy, {best_copepod_recall:.1f}% copepod recall
ODE state: {ODE_STATE_DIM}-dim, bottleneck: {BOTTLENECK_DIM}-dim
""")

print("Done!")









# # V2: correct matching with Matlab

# """
# =============================================================================
# Tiny Neural ODE Classifier — Verification-Friendly Version for TNODEV
# =============================================================================
# Run:  python pyopia_neural_ode_tiny_verification.py

# Architecture:
#   Pre-ODE:  Conv2d(3->2) -> BN -> ReLU -> Conv2d(2->2,s=2) -> BN -> ReLU -> Flatten
#             Input: 8x8x3  ->  Output: 4x4x2 = 32 dimensions

#   ODE:      dx/dt = W2 * ReLU(W1*x + b1) + b2    (FC bottleneck)
#             W1: 8x32,  W2: 32x8
#             Maps to System_description.m case 19

#   Post-ODE: FC(32->64) -> ReLU -> FC(64->7)

# ODE state dimension: 32 (vs 676 for MNIST)
# =============================================================================
# """

# import os
# import pathlib
# import numpy as np
# import matplotlib
# matplotlib.use("Agg")
# import matplotlib.pyplot as plt
# import seaborn as sns
# import pandas as pd

# import torch
# import torch.nn as nn
# import torch.nn.functional as F
# import torch.optim as optim
# from torch.utils.data import Dataset, DataLoader
# from torchvision import transforms
# from sklearn.model_selection import train_test_split
# import sklearn.metrics
# import scipy.io as sio

# from torchdiffeq import odeint, odeint_adjoint

# SEED = 42
# np.random.seed(SEED)
# torch.manual_seed(SEED)
# torch.cuda.manual_seed_all(SEED)

# DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")
# print(f"Using device: {DEVICE}")
# if DEVICE.type == "cuda":
#     print(f"GPU: {torch.cuda.get_device_name(0)}")


# # ═══════════════════════════════════════════════════════════════════════════
# #  Configuration — tiny verification model
# # ═══════════════════════════════════════════════════════════════════════════
# IMG_SIZE = 16            # small input for tractable verification (8 or 16 or 32)
# N_FILTERS = 4           # minimal filters -> (2: state_dim = 2 x 4 x 4 = 32) or (4: state_dim = 4 x 8 x 8 = 256)
# BOTTLENECK_DIM = 16      # FC bottleneck in ODE dynamics --> 8 or 16
# NUM_EPOCHS = 300        # more epochs since model is small
# LEARNING_RATE = 0.01
# BATCH_SIZE = 128
# ODE_TOL = 1e-3
# USE_ADJOINT = True
# SOLVER = "dopri5"

# # Derived
# SPATIAL_AFTER_DOWNSAMPLE = IMG_SIZE // 2  # 4
# ODE_STATE_DIM = N_FILTERS * SPATIAL_AFTER_DOWNSAMPLE * SPATIAL_AFTER_DOWNSAMPLE  # 32

# print(f"\n--- Tiny Verification Model Config ---")
# print(f"Image size:     {IMG_SIZE}x{IMG_SIZE}")
# print(f"Filters:        {N_FILTERS}")
# print(f"ODE state dim:  {ODE_STATE_DIM}")
# print(f"Bottleneck dim: {BOTTLENECK_DIM}")
# print(f"ODE dynamics:   dx/dt = W2 * ReLU(W1*x + b1) + b2")
# print(f"                W1: {BOTTLENECK_DIM}x{ODE_STATE_DIM}")
# print(f"                W2: {ODE_STATE_DIM}x{BOTTLENECK_DIM}")


# # ═══════════════════════════════════════════════════════════════════════════
# #  1. Download database
# # ═══════════════════════════════════════════════════════════════════════════
# print("\n--- Downloading PyOPIA particle database ---")
# import pyopia.exampledata

# db_path = pyopia.exampledata.get_classifier_database_from_pysilcam_blob(
#     download_directory="silcam_classification_database"
# )

# CLASS_LABELS = ["bubble", "copepod", "diatom_chain",
#                 "faecal_pellets", "oil", "oily_gas", "other"]
# NUM_CLASSES = len(CLASS_LABELS)
# COPEPOD_IDX = CLASS_LABELS.index("copepod")

# all_image_paths = []
# all_labels = []
# for idx, cl in enumerate(CLASS_LABELS):
#     paths = sorted(list(pathlib.Path(db_path, cl).glob("*.tiff")))
#     all_image_paths.extend(paths)
#     all_labels.extend([idx] * len(paths))
#     print(f"  {cl}: {len(paths)} images")
# print(f"Total: {len(all_image_paths)}")


# # ═══════════════════════════════════════════════════════════════════════════
# #  2. Dataset
# # ═══════════════════════════════════════════════════════════════════════════
# train_transform = transforms.Compose([
#     transforms.ToPILImage(),
#     transforms.Resize((IMG_SIZE, IMG_SIZE)),
#     transforms.RandomHorizontalFlip(),
#     transforms.RandomRotation(15),
#     transforms.ColorJitter(brightness=0.2, contrast=0.2),
#     transforms.ToTensor(),
# ])

# val_transform = transforms.Compose([
#     transforms.ToPILImage(),
#     transforms.Resize((IMG_SIZE, IMG_SIZE)),
#     transforms.ToTensor(),
# ])


# class ParticleDataset(Dataset):
#     def __init__(self, image_paths, labels, transform=None):
#         self.image_paths = image_paths
#         self.labels = labels
#         self.transform = transform

#     def __len__(self):
#         return len(self.image_paths)

#     def __getitem__(self, idx):
#         img = plt.imread(str(self.image_paths[idx]))
#         if img.dtype in (np.float32, np.float64):
#             img = (img * 255).clip(0, 255).astype(np.uint8)
#         if img.ndim == 2:
#             img = np.stack([img] * 3, axis=-1)
#         elif img.shape[-1] == 4:
#             img = img[..., :3]
#         if self.transform:
#             img = self.transform(img)
#         else:
#             img = transforms.ToTensor()(img)
#         return img, self.labels[idx]


# train_paths, val_paths, train_labels, val_labels = train_test_split(
#     all_image_paths, all_labels, test_size=0.2,
#     stratify=all_labels, random_state=SEED
# )

# train_ds = ParticleDataset(train_paths, train_labels, transform=train_transform)
# val_ds = ParticleDataset(val_paths, val_labels, transform=val_transform)

# train_loader = DataLoader(train_ds, batch_size=BATCH_SIZE, shuffle=True,
#                           num_workers=2, pin_memory=True, drop_last=True)
# val_loader = DataLoader(val_ds, batch_size=BATCH_SIZE, shuffle=False,
#                         num_workers=2, pin_memory=True)

# print(f"Train: {len(train_ds)}  |  Val: {len(val_ds)}")


# # ═══════════════════════════════════════════════════════════════════════════
# #  3. Model — Tiny verification-friendly architecture
# # ═══════════════════════════════════════════════════════════════════════════

# class ODEFuncFC(nn.Module):
#     """FC-based ODE dynamics: dx/dt = W2 * ReLU(W1*x + b1) + b2

#     Same structure as MNIST example in NNV.
#     Maps to System_description.m case 19.
#     """

#     def __init__(self, state_dim, bottleneck_dim):
#         super().__init__()
#         self._num_evals = 0
#         self.fc1 = nn.Linear(state_dim, bottleneck_dim)
#         self.fc2 = nn.Linear(bottleneck_dim, state_dim)

#     def forward(self, t, x):
#         self._num_evals += 1
#         h = F.relu(self.fc1(x))
#         return self.fc2(h)


# class ODEBlock(nn.Module):
#     def __init__(self, odefunc, solver="dopri5", rtol=1e-3, atol=1e-3,
#                  use_adjoint=False):
#         super().__init__()
#         self.odefunc = odefunc
#         self.solver = solver
#         self.rtol = rtol
#         self.atol = atol
#         self.use_adjoint = use_adjoint
#         self.register_buffer(
#             "integration_time",
#             torch.tensor([0.0, 1.0], dtype=torch.float32),
#         )

#     def forward(self, x):
#         solve_fn = odeint_adjoint if self.use_adjoint else odeint
#         out = solve_fn(
#             self.odefunc, x, self.integration_time,
#             method=self.solver, rtol=self.rtol, atol=self.atol,
#         )
#         return out[-1]


# class TinyODEnet(nn.Module):
#     """Tiny Neural ODE for verification.

#     Structure:
#       Pre-ODE:  Conv -> BN -> ReLU -> Conv(s=2) -> BN -> ReLU -> Flatten
#       ODE:      FC bottleneck dynamics (case 19 in System_description.m)
#       Post-ODE: FC -> ReLU -> FC
#     """

#     def __init__(self, in_channels=3, n_filters=4, state_dim=256,
#                  bottleneck_dim=16, num_classes=7, solver="dopri5",
#                  tol=1e-3, use_adjoint=False):
#         super().__init__()
#         self.state_dim = state_dim

#         self.pre_conv1 = nn.Conv2d(in_channels, n_filters, 3, stride=1,
#                                    padding=1, bias=True)
#         self.pre_bn1 = nn.BatchNorm2d(n_filters)

#         self.pre_conv2 = nn.Conv2d(n_filters, n_filters, 3, stride=2,
#                                    padding=1, bias=True)
#         self.pre_bn2 = nn.BatchNorm2d(n_filters)

#         self.ode_block = ODEBlock(
#             odefunc=ODEFuncFC(state_dim, bottleneck_dim),
#             solver=solver, rtol=tol, atol=tol,
#             use_adjoint=use_adjoint,
#         )

#         self.post_fc1 = nn.Linear(state_dim, 64)
#         self.post_fc2 = nn.Linear(64, num_classes)

#     def forward(self, x):
#         # Pre-ODE
#         x = F.relu(self.pre_bn1(self.pre_conv1(x)))
#         x = F.relu(self.pre_bn2(self.pre_conv2(x)))
#         x = x.flatten(1)

#         # ODE
#         x = self.ode_block(x)

#         # Post-ODE
#         x = F.relu(self.post_fc1(x))
#         x = self.post_fc2(x)
#         return x

#     def forward_with_intermediates(self, x):
#         """Forward pass returning intermediate outputs for debugging
#         PyTorch vs MATLAB mismatch."""
#         # Pre-ODE
#         x = F.relu(self.pre_bn1(self.pre_conv1(x)))
#         x = F.relu(self.pre_bn2(self.pre_conv2(x)))
#         pre_ode_4d = x.clone()           # [B, C, H, W] before flatten
#         x = x.flatten(1)
#         pre_ode_flat = x.clone()          # [B, 32] after flatten (PyTorch C-first order)

#         # ODE
#         x = self.ode_block(x)
#         post_ode = x.clone()             # [B, 32] after ODE integration

#         # Post-ODE
#         x = F.relu(self.post_fc1(x))
#         post_fc1_out = x.clone()         # [B, 64] after FC1+ReLU
#         x = self.post_fc2(x)
#         logits = x.clone()               # [B, 7] final output

#         return {
#             "pre_ode_4d": pre_ode_4d,
#             "pre_ode_flat": pre_ode_flat,
#             "post_ode": post_ode,
#             "post_fc1": post_fc1_out,
#             "logits": logits,
#         }


# def count_parameters(model):
#     return sum(p.numel() for p in model.parameters() if p.requires_grad)


# # ═══════════════════════════════════════════════════════════════════════════
# #  4. Training functions
# # ═══════════════════════════════════════════════════════════════════════════

# def train_one_epoch(model, loader, criterion, optimizer, scheduler=None):
#     model.train()
#     running_loss = 0.0
#     correct = 0
#     total = 0
#     for images, labels in loader:
#         images, labels = images.to(DEVICE), labels.to(DEVICE)
#         optimizer.zero_grad()
#         outputs = model(images)
#         loss = criterion(outputs, labels)
#         loss.backward()
#         torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
#         optimizer.step()
#         running_loss += loss.item() * images.size(0)
#         _, predicted = outputs.max(1)
#         total += labels.size(0)
#         correct += predicted.eq(labels).sum().item()
#     if scheduler is not None:
#         scheduler.step()
#     return running_loss / total, 100.0 * correct / total


# @torch.no_grad()
# def evaluate(model, loader, criterion):
#     model.eval()
#     running_loss = 0.0
#     correct = 0
#     total = 0
#     all_preds = []
#     all_labels = []
#     for images, labels in loader:
#         images, labels = images.to(DEVICE), labels.to(DEVICE)
#         outputs = model(images)
#         loss = criterion(outputs, labels)
#         running_loss += loss.item() * images.size(0)
#         _, predicted = outputs.max(1)
#         total += labels.size(0)
#         correct += predicted.eq(labels).sum().item()
#         all_preds.extend(predicted.cpu().numpy())
#         all_labels.extend(labels.cpu().numpy())
#     return (running_loss / total, 100.0 * correct / total,
#             np.array(all_preds), np.array(all_labels))


# def copepod_recall(y_true, y_pred, idx=1):
#     mask = (y_true == idx)
#     if mask.sum() == 0:
#         return 0.0
#     return 100.0 * (y_pred[mask] == idx).sum() / mask.sum()


# # ═══════════════════════════════════════════════════════════════════════════
# #  5. Train
# # ═══════════════════════════════════════════════════════════════════════════
# print("\n--- Building model ---")

# model = TinyODEnet(
#     in_channels=3,
#     n_filters=N_FILTERS,
#     state_dim=ODE_STATE_DIM,
#     bottleneck_dim=BOTTLENECK_DIM,
#     num_classes=NUM_CLASSES,
#     solver=SOLVER,
#     tol=ODE_TOL,
#     use_adjoint=USE_ADJOINT,
# ).to(DEVICE)

# print(f"Parameters: {count_parameters(model):,}")
# print(f"\nModel structure:")
# print(model)

# # Mild copepod weight
# loss_weights = torch.ones(NUM_CLASSES)
# loss_weights[COPEPOD_IDX] = 1.5
# criterion = nn.CrossEntropyLoss(weight=loss_weights.to(DEVICE), label_smoothing=0.1)

# optimizer = optim.SGD(model.parameters(), lr=LEARNING_RATE,
#                       momentum=0.9, weight_decay=1e-4)
# scheduler = optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=NUM_EPOCHS)

# print(f"\n--- Training ---")
# print(f"Epochs: {NUM_EPOCHS} | LR: {LEARNING_RATE} | Batch: {BATCH_SIZE}")

# history = {"train_loss": [], "train_acc": [], "val_loss": [], "val_acc": [],
#            "copepod_recall": []}
# best_val_acc = 0.0
# best_copepod_recall = 0.0

# for epoch in range(1, NUM_EPOCHS + 1):
#     model.ode_block.odefunc._num_evals = 0

#     train_loss, train_acc = train_one_epoch(
#         model, train_loader, criterion, optimizer, scheduler
#     )
#     val_loss, val_acc, y_pred, y_true = evaluate(model, val_loader, criterion)
#     cop_recall = copepod_recall(y_true, y_pred, COPEPOD_IDX)
#     nfe = model.ode_block.odefunc._num_evals

#     history["train_loss"].append(train_loss)
#     history["train_acc"].append(train_acc)
#     history["val_loss"].append(val_loss)
#     history["val_acc"].append(val_acc)
#     history["copepod_recall"].append(cop_recall)

#     if val_acc > best_val_acc:
#         best_val_acc = val_acc
#         best_copepod_recall = cop_recall
#         torch.save(model.state_dict(), "best_odenet_tiny.pth")

#     if epoch % 20 == 0 or epoch == 1:
#         print(f"Epoch {epoch:3d}/{NUM_EPOCHS}  "
#               f"Train: {train_loss:.4f} / {train_acc:.1f}%  |  "
#               f"Val: {val_loss:.4f} / {val_acc:.1f}%  |  "
#               f"Copepod: {cop_recall:.1f}%  |  "
#               f"NFE: {nfe}  |  Best: {best_val_acc:.1f}%")

#     if epoch % 50 == 0:
#         torch.save(model.state_dict(), f"checkpoint_tiny_epoch_{epoch}.pth")

# print(f"\nTraining complete!")
# print(f"Best overall accuracy: {best_val_acc:.2f}%")
# print(f"Copepod recall at best: {best_copepod_recall:.2f}%")


# # ═══════════════════════════════════════════════════════════════════════════
# #  6. Evaluation
# # ═══════════════════════════════════════════════════════════════════════════
# print("\n--- Loading best model and evaluating ---")
# model.load_state_dict(torch.load("best_odenet_tiny.pth", map_location=DEVICE))
# val_loss, val_acc, y_pred, y_true = evaluate(model, val_loader, criterion)

# # Confusion matrix
# cm = 100 * sklearn.metrics.confusion_matrix(y_true, y_pred, normalize="true")
# cm_df = pd.DataFrame(cm, index=CLASS_LABELS, columns=CLASS_LABELS)

# plt.figure(figsize=(9, 8))
# sns.heatmap(cm_df, annot=True, fmt=".1f", vmin=0, vmax=100, cmap="Blues")
# plt.xlabel("Predicted label")
# plt.ylabel("True label")
# plt.title(f"Tiny ODEnet (verification) - Val Accuracy: {val_acc:.2f}%")
# plt.tight_layout()
# plt.savefig("confusion_matrix_tiny.png", dpi=150)
# plt.show()

# print("\n--- Classification Report ---")
# print(sklearn.metrics.classification_report(
#     y_true, y_pred, target_names=CLASS_LABELS
# ))

# # Copepod detail
# cop_mask = (y_true == COPEPOD_IDX)
# cop_preds = y_pred[cop_mask]
# print(f"--- Copepod detail ---")
# print(f"Total copepod samples: {cop_mask.sum()}")
# print(f"Correctly classified:  {(cop_preds == COPEPOD_IDX).sum()}")
# print(f"Recall: {100 * (cop_preds == COPEPOD_IDX).sum() / cop_mask.sum():.1f}%")
# print(f"Misclassified as:")
# for i, cl in enumerate(CLASS_LABELS):
#     if i != COPEPOD_IDX:
#         n = (cop_preds == i).sum()
#         if n > 0:
#             print(f"  {cl}: {n} ({100*n/cop_mask.sum():.1f}%)")

# # Training curves
# fig, axes = plt.subplots(1, 3, figsize=(18, 5))
# axes[0].plot(history["train_loss"], label="Train", linewidth=1.5)
# axes[0].plot(history["val_loss"], label="Val", linewidth=1.5)
# axes[0].set_xlabel("Epoch"); axes[0].set_ylabel("Loss")
# axes[0].legend(); axes[0].set_title("Loss"); axes[0].grid(alpha=0.3)

# axes[1].plot(history["train_acc"], label="Train", linewidth=1.5)
# axes[1].plot(history["val_acc"], label="Val", linewidth=1.5)
# axes[1].set_xlabel("Epoch"); axes[1].set_ylabel("Accuracy (%)")
# axes[1].legend(); axes[1].set_title("Overall accuracy"); axes[1].grid(alpha=0.3)

# axes[2].plot(history["copepod_recall"], label="Copepod recall",
#              linewidth=2, color="#534AB7")
# axes[2].set_xlabel("Epoch"); axes[2].set_ylabel("Recall (%)")
# axes[2].legend(); axes[2].set_title("Copepod recall"); axes[2].grid(alpha=0.3)

# plt.tight_layout()
# plt.savefig("training_curves_tiny.png", dpi=150)
# plt.show()


# # ═══════════════════════════════════════════════════════════════════════════
# #  7. Export weights to .mat for TNODEV
# # ═══════════════════════════════════════════════════════════════════════════
# print("\n--- Exporting to .mat for TNODEV ---")

# model.eval()
# mat_data = {}

# # Pre-ODE Conv1: Conv2d(3->2, 3x3, s=1, pad=1)
# w = model.pre_conv1.weight.detach().cpu().numpy()   # [2, 3, 3, 3]
# b = model.pre_conv1.bias.detach().cpu().numpy()      # [2]
# mat_data["pre_conv1_W"] = np.transpose(w, (2, 3, 1, 0))  # -> [3, 3, 3, 2] MATLAB format
# mat_data["pre_conv1_b"] = b
# print(f"pre_conv1_W: {mat_data['pre_conv1_W'].shape}")

# # Pre-ODE BN1: BatchNorm2d(2)
# mat_data["pre_bn1_weight"] = model.pre_bn1.weight.detach().cpu().numpy()
# mat_data["pre_bn1_bias"] = model.pre_bn1.bias.detach().cpu().numpy()
# mat_data["pre_bn1_running_mean"] = model.pre_bn1.running_mean.detach().cpu().numpy()
# mat_data["pre_bn1_running_var"] = model.pre_bn1.running_var.detach().cpu().numpy()
# mat_data["pre_bn1_eps"] = model.pre_bn1.eps

# # Pre-ODE Conv2: Conv2d(2->2, 3x3, s=2, pad=1)
# w = model.pre_conv2.weight.detach().cpu().numpy()   # [2, 2, 3, 3]
# b = model.pre_conv2.bias.detach().cpu().numpy()      # [2]
# mat_data["pre_conv2_W"] = np.transpose(w, (2, 3, 1, 0))  # -> [3, 3, 2, 2]
# mat_data["pre_conv2_b"] = b
# print(f"pre_conv2_W: {mat_data['pre_conv2_W'].shape}")

# # Pre-ODE BN2: BatchNorm2d(2)
# mat_data["pre_bn2_weight"] = model.pre_bn2.weight.detach().cpu().numpy()
# mat_data["pre_bn2_bias"] = model.pre_bn2.bias.detach().cpu().numpy()
# mat_data["pre_bn2_running_mean"] = model.pre_bn2.running_mean.detach().cpu().numpy()
# mat_data["pre_bn2_running_var"] = model.pre_bn2.running_var.detach().cpu().numpy()
# mat_data["pre_bn2_eps"] = model.pre_bn2.eps

# # ODE dynamics: W1, b1, W2, b2
# # Maps to System_description.m case 19:
# #   h = W1 * x + b1        (compress to bottleneck)
# #   h_relu = max(0, h)      (ReLU on bottleneck neurons)
# #   dx = W2 * h_relu + b2   (expand back to state dim)
# ode_fc1_W = model.ode_block.odefunc.fc1.weight.detach().cpu().numpy()  # [8, 32]
# ode_fc1_b = model.ode_block.odefunc.fc1.bias.detach().cpu().numpy()    # [8]
# ode_fc2_W = model.ode_block.odefunc.fc2.weight.detach().cpu().numpy()  # [32, 8]
# ode_fc2_b = model.ode_block.odefunc.fc2.bias.detach().cpu().numpy()    # [32]

# mat_data["ode_W1"] = ode_fc1_W
# mat_data["ode_b1"] = ode_fc1_b
# mat_data["ode_W2"] = ode_fc2_W
# mat_data["ode_b2"] = ode_fc2_b

# print(f"ode_W1: {ode_fc1_W.shape}  (bottleneck x state)")
# print(f"ode_b1: {ode_fc1_b.shape}")
# print(f"ode_W2: {ode_fc2_W.shape}  (state x bottleneck)")
# print(f"ode_b2: {ode_fc2_b.shape}")

# # Post-ODE FC1: FC(32->64)
# mat_data["post_fc1_W"] = model.post_fc1.weight.detach().cpu().numpy()  # [64, 32]
# mat_data["post_fc1_b"] = model.post_fc1.bias.detach().cpu().numpy()    # [64]
# print(f"post_fc1_W: {mat_data['post_fc1_W'].shape}")

# # Post-ODE FC2: FC(64->7)
# mat_data["post_fc2_W"] = model.post_fc2.weight.detach().cpu().numpy()  # [7, 64]
# mat_data["post_fc2_b"] = model.post_fc2.bias.detach().cpu().numpy()    # [7]
# print(f"post_fc2_W: {mat_data['post_fc2_W'].shape}")

# # Metadata
# mat_data["img_size"] = np.array([IMG_SIZE])
# mat_data["n_filters"] = np.array([N_FILTERS])
# mat_data["num_classes"] = np.array([NUM_CLASSES])
# mat_data["ode_state_dim"] = np.array([ODE_STATE_DIM])
# mat_data["bottleneck_dim"] = np.array([BOTTLENECK_DIM])
# mat_data["ode_t_final"] = np.array([1.0])
# mat_data["class_labels_str"] = ",".join(CLASS_LABELS)
# mat_data["copepod_class_idx_matlab"] = np.array([COPEPOD_IDX + 1])  # 1-indexed

# sio.savemat("odenet_tiny_copepod_weights.mat", mat_data)
# print(f"\nSaved weights to: odenet_tiny_copepod_weights.mat")


# # ═══════════════════════════════════════════════════════════════════════════
# #  8. Export CORRECTLY CLASSIFIED copepod images WITH PyTorch intermediates
# # ═══════════════════════════════════════════════════════════════════════════
# print("\n--- Exporting verified copepod images with PyTorch intermediates ---")
# model.eval()

# correct_images = []          # [C, H, W] tensors
# correct_logits = []          # final output logits [7]
# correct_pre_ode_flat = []    # 32-dim vector after flatten (PyTorch C-first order)
# correct_post_ode = []        # 32-dim vector after ODE integration

# for i in range(len(val_paths)):
#     if val_labels[i] != COPEPOD_IDX:
#         continue

#     img = plt.imread(str(val_paths[i]))
#     if img.dtype in (np.float32, np.float64):
#         img = (img * 255).clip(0, 255).astype(np.uint8)
#     if img.ndim == 2:
#         img = np.stack([img] * 3, axis=-1)
#     elif img.shape[-1] == 4:
#         img = img[..., :3]

#     tensor = val_transform(img).unsqueeze(0).to(DEVICE)
#     with torch.no_grad():
#         intermediates = model.forward_with_intermediates(tensor)
#         pred = intermediates["logits"].argmax(dim=1).item()

#     if pred == COPEPOD_IDX:  # only export correctly classified images
#         correct_images.append(tensor[0].cpu().numpy())                    # [C, H, W]
#         correct_logits.append(intermediates["logits"][0].cpu().numpy())    # [7]
#         correct_pre_ode_flat.append(intermediates["pre_ode_flat"][0].cpu().numpy())  # [32]
#         correct_post_ode.append(intermediates["post_ode"][0].cpu().numpy())          # [32]

#     if len(correct_images) >= 20:
#         break

# num_exported = len(correct_images)
# images_array = np.stack(correct_images, axis=0)              # [N, C, H, W]
# logits_array = np.stack(correct_logits, axis=0)              # [N, 7]
# pre_ode_flat_array = np.stack(correct_pre_ode_flat, axis=0)  # [N, 32]
# post_ode_array = np.stack(correct_post_ode, axis=0)          # [N, 32]

# # Convert images to MATLAB format [H, W, C, N]
# images_matlab = np.transpose(images_array, (2, 3, 1, 0))

# sio.savemat("copepod_test_images_tiny.mat", {
#     "images": images_matlab,                              # [H, W, C, N] for MATLAB
#     "labels": np.ones(num_exported) * (COPEPOD_IDX + 1),  # 1-indexed for MATLAB
#     "num_samples": np.array([num_exported]),
#     "img_size": np.array([IMG_SIZE]),
#     "class_name": "copepod",
#     # PyTorch reference outputs for debugging MATLAB mismatch
#     "pytorch_logits": logits_array,                # [N, 7]  — final class scores
#     "pytorch_pre_ode_flat": pre_ode_flat_array,    # [N, 32] — after flatten (PyTorch order!)
#     "pytorch_post_ode": post_ode_array,            # [N, 32] — after ODE integration
#     "pytorch_pre_ode_4d_shape": np.array([N_FILTERS, SPATIAL_AFTER_DOWNSAMPLE,
#                                            SPATIAL_AFTER_DOWNSAMPLE]),  # [C, H, W] = [2, 4, 4]
# })

# print(f"Exported {num_exported} correctly-classified copepod images")
# print(f"PyTorch logits for image 1:       {logits_array[0]}")
# print(f"PyTorch prediction for image 1:   {CLASS_LABELS[logits_array[0].argmax()]}")
# print(f"PyTorch pre-ODE flat (first 8):   {pre_ode_flat_array[0][:8]}")
# print(f"PyTorch post-ODE (first 8):       {post_ode_array[0][:8]}")


# # ═══════════════════════════════════════════════════════════════════════════
# #  9. Save full checkpoint
# # ═══════════════════════════════════════════════════════════════════════════
# torch.save({
#     "model_state_dict": model.state_dict(),
#     "class_labels": CLASS_LABELS,
#     "img_size": IMG_SIZE,
#     "n_filters": N_FILTERS,
#     "state_dim": ODE_STATE_DIM,
#     "bottleneck_dim": BOTTLENECK_DIM,
#     "solver": SOLVER,
#     "tol": ODE_TOL,
#     "use_adjoint": USE_ADJOINT,
#     "best_val_acc": best_val_acc,
#     "best_copepod_recall": best_copepod_recall,
#     "history": history,
# }, "odenet_tiny_full.pth")


# # ═══════════════════════════════════════════════════════════════════════════
# #  10. Print summary
# # ═══════════════════════════════════════════════════════════════════════════
# print("\n" + "=" * 65)
# print("  TNODEV MATLAB INTEGRATION")
# print("=" * 65)
# print(f"""
# Files created:
#   1. odenet_tiny_copepod_weights.mat  — network weights
#   2. copepod_test_images_tiny.mat     — {num_exported} verified copepod images
#      + pytorch_logits                — PyTorch final output [N,7] (ground truth)
#      + pytorch_pre_ode_flat          — 32-dim flatten vector [N,32] (PyTorch order!)
#      + pytorch_post_ode              — 32-dim after ODE [N,32]

# To debug MATLAB mismatch in run_copepod_quick.m:

#   load('copepod_test_images_tiny.mat');
#   fprintf('PyTorch logits:       %s\\n', mat2str(pytorch_logits(1,:), 4));
#   fprintf('PyTorch pre-ODE flat: %s\\n', mat2str(pytorch_pre_ode_flat(1,:), 4));
#   fprintf('PyTorch post-ODE:     %s\\n', mat2str(pytorch_post_ode(1,:), 4));

#   % Compare at each stage:
#   % 1. Sort both pre-ODE flat vectors — if same values in different order -> permutation
#   % 2. If pre-ODE flat values differ entirely -> conv/BN weight issue
#   % 3. If pre-ODE matches but post-ODE differs -> ODE integration issue

# Model: {best_val_acc:.1f}% accuracy, {best_copepod_recall:.1f}% copepod recall
# ODE state: {ODE_STATE_DIM}-dim, bottleneck: {BOTTLENECK_DIM}-dim
# """)

# print("Done!")