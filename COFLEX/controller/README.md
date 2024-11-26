# COFLEX: the Controller

The COFLEX Controller is a Simulink-based control scheme for wind turbines. This repository contains all the necessary files to use the COFLEX control scheme.

## Folder Content

- `init_controller.m`: MATLAB script to initialise the controller environment and load necessary parameters.
- `Controller.slx`: A Simulink model for the COFLEX controller, with inputs and output ports to the controller.
- `DISCON_FFFB_IEA15MW.slx`: The main Simulink model file for the COFLEX controller. If you need to build the controller as a `.dll`, press `CTRL+B` in this file.
- `discon.tlc`: Target Language Compiler (TLC) file for generating the controller code.
- `discon_main.c`: C implementation file for external controller execution.
- `discon_vc.tmf`: Configuration file for controller compilation.
- `LUT_for_satlimits.mat`: Lookup table for saturation limits.
- `LUT_for_setpoints.mat`: Lookup table for feedforward control setpoints.
- `LUT_for_WSE.mat`: Lookup table for the wind speed estimator.
- `Simulator.slx`: A Simulink model for simulating a simple model of a wind turbine with the COFLEX controller.
- `WindSpeed.slx`: A Simulink model to define wind speed profiles for simulation in `Simulator.slx`.
- `README.md`: The file you are reading.

## COFLEX Workflow

### Step 1: Initialise the Environment
Run the `init_controller.m` MATLAB script to initialise the COFLEX controller environment. This script:

1. Adds required paths.
2. Loads global and turbine-specific constants.
3. Loads or generates lookup tables for the wind speed estimator, feedforward setpoints, and saturation limits, based on COFLEXOpt outputs.
4. Use the `init_controller.m` script to modify these parameters, if needed:  
   - Initial Wind Speed (U0): Starting horizontal wind speed in m/s.
   - Initial Rotor Speed (Wr0): Initial value for the rotor rotational speed in rad/s.
   - Time Step (Control.DT).
   - Maximum Simulation Time (TMax).

### Step 2: Choose How to Use the Controller

The COFLEX controller can be used in one of the following ways, depending on your simulation environment:

1. **Build a Bladed-Style `.dll` File for HAWC2**  
   Use the `DISCON_FFFB_IEA15MW.slx` Simulink model to generate a Bladed-style `.dll` file. This file can then be used within HAWC2.  
   Check the example contained in the repository to adapt the input file of HAWC2 accordingly. In particular, use a text input file like `\test_IEA15MW_HAWC2\hawc2_turb_sims\IEA-15-240-RWT-Onshore\BladedDLL.par` for the constants needed by COFLEX usage.
2. **Run the Controller Directly in `Simulator.slx`**  
   Use the `Simulator.slx` Simulink model to simulate the turbine system directly with the COFLEX controller. This option provides a complete simulation setup, integrating a turbine model, a wind speed generator and the controller for testing and evaluation.
3. **Integrate the Controller with OpenFAST**   
   Reference the `Controller.slx` model in your own Simulink parent model linked with OpenFAST. This approach requires setting up your simulation environment with OpenFAST's Simulink interface and linking the COFLEX controller as part of the larger system model. Ensure that the necessary inputs and outputs are properly connected for OpenFAST integration. Be aware that you should *always* activate the Beamdyn module in OpenFAST to allow for the torsion DoF on blade sections, which plays a crucial role in performance of flexible wind turbines.