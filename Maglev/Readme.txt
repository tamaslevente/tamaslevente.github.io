README.txt

---

Project Title:
PID Controller for Magnetic Levitation System using Simulink and Arduino

---

Overview
This project implements a PID controller for a magnetic levitation system using MATLAB Simulink and Arduino. The controller is configured and deployed via the Simulink model `PID_Arduino3.slx`. Experimental data is stored in `.mat` files, and workspace parameters are initialized using the `parameters.m` script.

---

Project Files

| File Name           | Description                                                                 |
|---------------------|-----------------------------------------------------------------------------|
| PID_Arduino3.slx    | Simulink model of the control system. Used for real-time implementation.    |
| parameters.m        | MATLAB script for initializing model parameters and workspace variables.    |
| TL_logs.mat         | Experimental data log using the TL (Trial-and-Error) tuning method.         |
| ZN_PI_logs.mat      | Experimental data log for Ziegler–Nichols PI tuning.                        |
| ZN_PID_logs.mat     | Experimental data log for Ziegler–Nichols PID tuning.                       |

---

Instructions

1. Open MATLAB.

2. Run `parameters.m`
   - This script will initialize the workspace variables required for the Simulink model.

3. Load and Run Simulink Model
   - Open `PID_Arduino3.slx` in Simulink.
   - Configure the target hardware for Arduino (Tools > Run on Target Hardware > Prepare).
   - Build and deploy the model to the Arduino board.
   - Monitor real-time data if desired using scopes or logging blocks.

4. View Measurement Data
   - Load the `.mat` files in MATLAB using:
     load('TL_logs.mat');
     load('ZN_PI_logs.mat');
     load('ZN_PID_logs.mat');
   - Analyze data using MATLAB plotting tools or scripts.

---

Requirements

- MATLAB with Simulink
- Simulink Support Package for Arduino Hardware
- Arduino board (e.g., Due) connected to the magnetic levitation hardware

---

Notes

- Ensure all hardware connections are secure before deploying the model.
- Tune PID parameters as needed in the Simulink block configuration or via `parameters.m`.
