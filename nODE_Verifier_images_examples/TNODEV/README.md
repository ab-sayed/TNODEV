## TNODEV: Toolbox for Neural ODE Verification
This repository contains the matlab codes for our tool paper: "TNODEV: Toolbox for Neural ODE Verification"


## Prerequisites
- [CORA](https://tumcps.github.io/CORA/):  Continuous Reachability Analyzer toolbox
- [NNV](https://github.com/verivital/nnv/tree/master): Neural Network Verification toolbox


## Folder structure
```
TNODEV ToolBox
├── Documentation
│ ├── TNODEV_User Manual
│ └── NeurReps25_Paper
├── Examples
│ ├── Spiral_non [Status: Done]
│ ├──── spiral_non_CORA_RT.m
│ ├── FPA [Status: Done]
│ ├──── fpa_CORA_RT.m
│ ├── ACC [Status: Done]
│ ├──── acc_CORA_RT.m
│ ├──── acc_node_cora.m
│ ├──── controller_5_20.mat
│ ├──── dynamicsACC.m
│ ├──── plant_3rd_order_tanh.mat
│ ├──── reach_acc_TNODEV.m
│ ├──── results_tanhplant.mat
│ ├──── tanh_plant.m
│ ├──── acc_tnodev_distance.pdf --> Resulting saved image in pdf format
│ ├──── acc_tnodev_distance.png --> Resulting saved image in png format
│ └── MNIST [Status:On-going]
├── Input_Files 
│ ├── UP_Jacobian_2ndOrder_Bounds.m --> Used in OA_4
│ ├── UP_Jacobian_Bounds.m --> Used in OA_3_et_4
│ ├── UP_Jacobian_Function.m --> Used in OA_4
│ ├── UP_Jacobian_Signs.m --> Used in OA_3
│ ├── UP_Sensitivity_Bounds.m --> Used in OA_4
│ └── UP_Sensitivity_Signs.m --> Used in OA_4
├── OA_Methods
│ ├── OA_3_CT_Mixed_Monotonicity.m
│ ├── OA_4_CT_Sampled_data_MM.m
│ └── TIRA.m
├── Test_suite 
├── Utilities
├── Verifier_MAIN_CALL.m
```