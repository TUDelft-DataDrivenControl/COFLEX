# COFLEX: **CO**ntrol scheme for large and **FLEX**ible wind turbines

## Overview and Key Features of COFLEX
This repository provides the source code and test examples for **COFLEX**, a novel control scheme tailored for large, flexible wind turbines.  
COFLEX is composed of two key modules:

1. **COFLEXOpt**  
   A set point optimisation module that determines the optimal rotor speed, collective pitch angle, generator torque set points, and operating points across the entire operating range of the turbine. By incorporating blade flexibility into the optimisation process, COFLEXOpt aims for accurate evaluation of optimal aerodynamic performance while ensuring that structural constraints—such as blade tip deflection—are addressed.

2. **Feedforward-Feedback Controller**  
   This module contains a controller that tracks the optimised set points provided by COFLEXOpt using wind speed estimates. It combines feedforward control, which targets COFLEXOpt set points, with feedback corrections to mitigate modelling errors and maintain closed-loop stability. A set point smoothing technique ensures smooth transitions between partial and full load regions.

**COFLEX** has been validated on the **IEA 15 MW RWT**, and tested with **HAWC2**. An **OpenFAST** implementation has been made available additionally to encourage community users to test it.

## Installation

To set up and run the COFLEX framework, ensure the following software is installed and configured:

1. **MATLAB with Simulink**  
   - COFLEX was tested with the latest version of MATLAB and Simulink (R2024b). Ensure that Simulink is installed and functional.

2. **ACADOS and CasADi**  
   - COFLEXOpt uses CasADi for symbolic function and optimisation routines. The authors tested CasADi with ACADOS installation. However, the code will likely also work if CasADi is downloaded from the primary website. Follow the official installation guides:  
     - [ACADOS Installation](https://github.com/acados/acados)  
     - [CasADi for MATLAB](https://web.casadi.org/get/)  

3. **Optional Dependencies for Simulation**  
   - **HAWC2**  
     - To run simulations using HAWC2, you will need a license.  
     - Refer to the [HAWC2 Website](https://tools.windenergy.dtu.dk/home/default.html) for installation and licensing (academic license exists).  

   - **OpenFAST**  
     - OpenFAST is open-source and can be downloaded [here](https://github.com/OpenFAST/openfast).  
     - COFLEX has been tested with OpenFAST only through Simulink. Ensure the OpenFAST Simulink interface is properly configured if you plan to use it.
  
## Repository Structure

The COFLEX repository is organised as follows:

- **`COFLEX/`**  
  
  - **`COFLEXOpt/`**  
    - **`main_coflexopt.m`**: This is the set point optimisation script for determining the optimal rotor speed, collective pitch angle, and operating points.
    - **`functions/`**: Contains utility functions for the optimisation process.
  - **`controller/`**: This folder contains COFLEX Simulink controller model and other useful Simulink models.
  - **`data/`**  
    Stores input data and tables required for simulations and optimisation.  
    - **`inputs/`**: Turbine-specific datasets of performance, serving as input files for COFLEXOpt.  
    - **`lookup_tables/`**: Precomputed tables for the wind speed estimator, serving as input files for COFLEX.  
    - **`sat_limits/`**: Saturation limits, serving as input files for COFLEX.  
    - **`set_points/`**: Set points for the turbine, serving as input files for COFLEX.  
  
  - **`DRC_SimulinkLibrary/`**  
    A Simulink library containing reusable blocks and functions for COFLEX.

- **`test_IEA15MW_HAWC2/`**  
  Files to run simulations of the IEA 15 MW RWT using the HAWC2 model.  
  - **`IEA-15-240-RWT/`**: Baseline model files.  
  - **`IEA-15-240-RWT-Onshore/`**: Onshore configuration files, also contains pre-compiled COFLEX `.dll` files in the **`control/`** folder.

- **`test_IEA15MW_OpenFAST_Simulink/`**  
  Scripts and files to run simulations of the IEA 15 MW turbine using the OpenFAST model in Simulink.  
  - **`COFLEX/`**: COFLEX configuration specific to this OpenFAST test.  
  - **`OF_simulation/`**: Predefined simulation setup.  
  
## How to Use COFLEX

### 1. Generate Set Points and Saturation Limits

To compute optimised set points and operational constraints:

1. Navigate to the **`COFLEX/COFLEXOpt/`** folder.
2. Open and run **`main_coflexopt.m`**.

#### Inputs Required
- The script relies on turbine-specific data and performance interpolators generated using the function **`loadturbineconstants.m`**. This function is located in the **`COFLEX/DRC_SimulinkLibrary/`** folder. It is used to:
    - Load values for turbine characteristics.
    - Load precomputed datasets for performance metrics of wind turbines, calculated on grids with three independent variables "rotor speed", "wind speed", "collective pitch angle" otherwise represented by $(\omega, V, \beta)$.
    - Create multi-variate B-spline interpolators, representing performance metrics in a $(\omega, V, \beta)$ space.
    - We generated the datasets with HAWCStab2 for the IEA 15 MW RWT (the datasets are provided in this repository in **`data/inputs/`** folder).
    - You can replace these datasets with your own results from different tools, bear in mind that you will need to adapt the dataset loading routines.

#### Procedure
The workflow of **`main_coflexopt.m`** is illustrated in the README that can be found in **`COFLEX/COFLEXOpt/`**

#### Outputs Generated
- Lookup tables for generator torque, blade pitch angle, and rotor speed as functions of wind speed.
- Saturation limits for blade pitch angle.
  
### 2. Run the Simulink Model

The COFLEX controller is implemented as a modular Simulink model.

1. Navigate to the **`COFLEX/controller/`** folder.
2. Open the script **`init_controller.m`**
3. The workflow of **`init_controller.m`** is illustrated in the README that can be found in **`COFLEX/controller/`**

#### Features of the Simulink Model
The Simulink model is organised into clearly defined modules:
- Feedforward and feedback controller
- Set point smoother
- Wind speed estimator
  
### 3. Test with Provided Examples

Example configurations for the **IEA 15 MW RWT** are available in the repository:
- **`test_IEA15MW_HAWC2/`**: For simulations using HAWC2.
- **`test_IEA15MW_OpenFAST_Simulink/`**: For simulations using OpenFAST integrated with Simulink.
  
#### **IMPORTANT**:
- If you are working with a different turbine, you must replace the datasets and update COFLEX accordingly.  

## Authors and Contact Information
- Guido Lazzerini
- Jacob Deleuran Grunnet
- Tobias Gybel Hovgaard
- Vasu Datta Madireddi
- Delphine De Tavernier
- Sebastiaan Paul Mulders  

For any questions or feedback, please don't hesitate to contact **Guido Lazzerini** at g.lazzerini@tudelft.nl
  
## License
This project is licensed under the Apache License, Version 2.0. See the LICENSE file for more information.
  
## Referencing
If you use COFLEX, please reference the work behind it, citing: https://doi.org/10.5194/wes-2024-151  
You can use the predefined BIBTEX citation:
```
@Article{wes-2024-151,
AUTHOR = {Lazzerini, G. and Deleuran Grunnet, J. and Gybel Hovgaard, T. and Caponetti, F. and Datta Madireddi, V. and De Tavernier, D. and Mulders, S. P.},
TITLE = {COFLEX: A novel set point optimiser and feedforward-feedback control scheme for large flexible wind turbines},
JOURNAL = {Wind Energy Science Discussions},
VOLUME = {2024},
YEAR = {2024},
PAGES = {1--35},
URL = {https://wes.copernicus.org/preprints/wes-2024-151/},
DOI = {10.5194/wes-2024-151}
}
```
To reference COFLEX source code directly, use the following DOI: [![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.14051517.svg)](https://doi.org/10.5281/zenodo.14051517)
  
## Acknowledgments
The authors acknowledge the contribution to the development of COFLEX by Markel Meseguer San Martin and Ebbe Nielsen from Shanghai Electric Wind Power Generation European Innovation Center. This research received funding from TKI Wind op 610 Zee, under grant "TKITOE WOZ 2309 TUDELFT COCOFLEX". For a complete list of works that inspired COFLEX and its features, see the references in the related paper.
  