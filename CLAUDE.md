# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

EXCITE Satellite AOCS Simulation - A complete Attitude and Orbital Control System (AOCS) simulation for the EXCITE CubeSat 12U satellite using the Basilisk framework (http://hanspeterschaub.info/basilisk).

The simulation implements:
- Detailed dynamic models (environment, actuators, sensors)
- Advanced Flight Software (FSW) algorithms for Guidance, Navigation, and Control (GNC)
- Autonomous mission logic based on Finite State Machine (FSM)

## Core Architecture

The project uses Basilisk's hybrid architecture:
- **Core algorithms**: Written in C/C++ for performance (located in `modules/`)
- **Simulation framework**: Python scripts for configuration and orchestration (located in `simulation/`)
- **Custom modules**: QUEST and SMEKF are custom C implementations integrated into Basilisk

### File Organization

```
simulation/
  ├── EXCITE_scenario.py    # Main simulation orchestrator
  ├── EXCITE_Dynamics.py    # Physical models (BSKDynamicModels)
  ├── EXCITE_Fsw.py         # Flight software algorithms (BSKFswModels)
  └── EXCITE_Plotting.py    # Plotting utilities

modules/
  ├── SMEKF.c              # Custom Sequential Multiplicative EKF
  └── questAttDet.c        # Custom QUEST attitude determination

docs/
  ├── 0_context.md         # Architecture and development context
  └── SMEKF_Code.txt       # SMEKF implementation notes
```

## Running the Simulation

```bash
# Activate virtual environment (if using one)
source venv_alberto/bin/activate

# Run main simulation
python3 simulation/EXCITE_scenario.py
```

The simulation will:
- Execute a 24-hour mission scenario
- Print mission events to console (mode transitions, eclipses, etc.)
- Generate performance plots at the end

## Key Architectural Concepts

### Process and Task Structure
- **DynamicsProcess**: Runs physical simulation (spacecraft dynamics, environment, sensors)
- **FSWProcess**: Runs flight software (navigation, control algorithms, FSM logic)
- **Task rates**: Currently both set to 0.1s (10Hz). FSW rate should be ≤ Dynamics rate to ensure messages are written before being read.

### Message Passing
Basilisk uses a message-passing architecture:
- Dynamics modules write sensor/state messages
- FSW modules read these messages and write command messages
- Dynamics modules read commands and update actuator states

### Mission Phases (FSM States)
1. **detumbling**: Initial stabilization using B-dot controller
2. **deployment**: Solar panel deployment
3. **sunSafePoint**: Point solar panels toward Sun
4. **GSPoint**: Point S-band antenna toward Pisa Ground Station
5. **payloadModeA_ReconfANT**: ReconfANT experiment (point X-axis)
6. **payloadModeB_GPUIOT**: GPU IoT experiment (point -X axis)
7. **imagingMode**: Nadir pointing for IM200 camera imaging

### Navigation Algorithms (Current Development Focus)

**QUEST (questAttDet.c)**
- Solves Wahba's problem: determines attitude from two vector measurements
- Inputs: Sun direction + Magnetometer readings (both in Body and Inertial frames)
- Status: Currently being debugged - convergence issues
- Location in FSW: `EXCITE_Fsw.py` line ~10 (imported), configured in `SetQUEST()` method

**SMEKF (SMEKF.c)**
- Sequential Multiplicative Extended Kalman Filter for attitude estimation
- Fuses: Gyroscope + Star Tracker + QUEST output
- Status: Currently disabled to allow QUEST debugging
- Debugging strategy: Re-enable using only Star Tracker (reliable) without QUEST input
- State vector: 6 states (3 attitude error, 3 gyro bias)

### Control Architecture

**Outer Loop: MRP Steering**
- Generates rate commands with angular velocity limits
- Located in: `EXCITE_Fsw.py` - `mrpSteering` module

**Inner Loop: Rate Servo**
- Tracks desired angular velocity
- Outputs torque commands for reaction wheels
- Located in: `EXCITE_Fsw.py` - `rateServo` module

**Momentum Management**
- Reaction wheel desaturation using magnetorquers (MTB)
- B-dot controller for initial detumbling

### Hardware Models (Dynamics)

**Actuators**
- 4x Reaction Wheels (CubeSpace CW0162) in pyramidal configuration
- 3x Magnetorquers for desaturation/detumbling
- 1x H2O2 chemical thruster (UniPi IOD) for orbital maneuvers
- 4x Deployable solar panels

**Sensors**
- Star Tracker (high accuracy attitude)
- Coarse Sun Sensors (CSS)
- Magnetometer (TAM)
- Custom IMU with bias drift model
- GPS/GNSS (via SimpleNav)

**Environment**
- Gravity: Earth, Sun, Moon
- Gravity gradient torque
- Atmospheric drag (exponential model)
- Solar radiation pressure (SRP)
- Residual magnetic disturbance
- Eclipse modeling

## Development Notes

### Important Constraints
- NEVER update git config unless explicitly requested
- Custom modules (SMEKF, QUEST) are not in official Basilisk docs - they are project-specific C implementations
- The simulation requires Basilisk framework to be properly installed

### Common Debugging Workflow
1. Check message connections in FSW and Dynamics setup methods
2. Verify task scheduling (FSW rate ≤ Dynamics rate)
3. Use console print statements in scenario for state debugging
4. Check EXCITE_Plotting.py for visualization of results
5. Review `docs/0_context.md` for algorithm implementation details

### Key Configuration Points
- Initial conditions: Set in `scenario_EXCITE.__init__()` in EXCITE_scenario.py
- Spacecraft properties: Mass, inertia in `EXCITE_Dynamics.py` - `SetSpacecraftHub()`
- Control gains: In respective FSW module setup methods (e.g., `SetMRPSteering()`)
- Mission timeline: FSM event definitions in `EXCITE_Fsw.py` (e.g., `detumblingEndTime`)

### Python Dependencies
- Basilisk framework (primary dependency)
- NumPy
- Matplotlib

### File Naming Conventions
- `EXCITE_*`: Main simulation files
- `BSK_*`: Basilisk framework utilities
- Module objects use CamelCase (e.g., `scObject`, `simpleNavObject`)
- Messages use `*Msg` suffix (e.g., `navStateOutMsg`)

## Current Development Status

The project is actively debugging the QUEST algorithm integration. The SMEKF is temporarily disabled to isolate QUEST issues. Once QUEST converges correctly, SMEKF will be re-enabled to fuse Star Tracker, Gyroscope, and QUEST outputs for optimal attitude estimation.
