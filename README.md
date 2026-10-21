
# TNODEV: Toolbox for Neural ODE Verification

This repository contains the MATLAB codes for TNODEV toolbox:

- Paper: [TNODEV: Toolbox for Neural ODE Verification](https://openreview.net/forum?id=3lbdS0A1N5)


## Prerequisites
- [CORA](https://tumcps.github.io/CORA/):  Continuous Reachability Analyzer toolbox
- [NNV](https://github.com/verivital/nnv/tree/master): Neural Network Verification toolbox

## Folder structure
```
TNODEV ToolBox
├── Examples
│ ├── Spiral linear
│ ├── Spiral nonlinear
│ ├── FPA
│ ├── Cartpole
│ ├── ACC linear
│ ├── ACC nonlinear
│ └── MNIST 
├── Input_Files
│ ├── UP_Jacobian_2ndOrder_Bounds.m
│ ├── UP_Jacobian_Bounds.m 
│ ├── UP_Jacobian_Function.m
│ ├── UP_Jacobian_Signs.m 
│ ├── UP_Sensitivity_Bounds.m 
│ └── UP_Sensitivity_Signs.m 
├── OA_Methods
│ ├── OA_3_CT_Mixed_Monotonicity.m
│ └── TIRA.m
├── LICENSE.txt
├── README.md
└── Verifier_MAIN_CALL.m
```
