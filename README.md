#  MPC-PID Mobile Robot Controller

This project implements a hybrid **Model Predictive Control (MPC)** and **PID control** scheme to guide a mobile robot along a predefined trajectory. The controller ensures smooth motion, accurate tracking, and robustness by combining the long-term planning capabilities of MPC with the responsiveness of PID controllers.

---

##  Control Architecture Overview

The overall control framework is structured as follows:

1. **Trajectory Planning (MPC):**
   - Predicts and plans optimal control inputs over a future horizon.
   - Minimizes deviations from the desired trajectory and control effort.
   - Ensures constraints (e.g., velocity limits) are respected.

2. **Velocity Tracking (PID):**
   - Tracks the velocity outputs from MPC using real-time feedback.
   - Compensates for model mismatches or disturbances using odometry.

3. **Wheel Speed Control:**
   - Converts velocity commands to individual wheel speeds (e.g., for a mecanum-wheeled robot).
   - Publishes commands to the robot’s actuators.

---

## 🗺️ Step-by-Step Execution

### 1. Controller Initialization
- Initializes ROS publishers and subscribers.
- Loads waypoints from a CSV file.
- Generates and resamples the reference trajectory.

### 2. Control Loop (`startControlLoop`)
- Runs at a fixed time step (`Controller_dt`).
- Repeatedly:
  - Estimates the robot's current state from odometry.
  - Updates the MPC inputs and solves the optimization problem.
  - Interpolates MPC outputs for the high-rate PID controller.
  - Updates and publishes wheel speeds.

### 3. MPC Optimization
- Predicts optimal control inputs (velocities) every few steps.
- Minimizes a cost function combining state errors and control effort.
- Considers system dynamics and velocity constraints.

### 4. Interpolation & PID Tracking
- Interpolates MPC velocity outputs into finer control steps.
- PID controllers track interpolated velocities in real time.

### 5. Actuation
- Converts linear/angular velocities into wheel speeds.
- Publishes commands for actuator-level execution.

### 6. Visualization & Logging
- Publishes MPC-predicted trajectory as ROS Markers in RViz.
- Logs state and reference data for post-analysis.

### 7. Safety Handling
- Defaults to a stop state if MPC fails or control bounds are exceeded.

---

##  Class Functionality

###  `MPC` Class
| Function | Description |
|---------|-------------|
| `MPC::MPC(...)` | Initializes system model, weights, and constraints. |
| `setWeights(...)` | Sets Q, R, and P matrices for cost function. |
| `setSystemModel()` | Defines the discrete robot dynamics model in CasADi. |
| `optimize(...)` | Solves the MPC problem using the current robot state. |
| `getPredictU()` | Retrieves optimal control inputs. |
| `getPredictX()` | Retrieves predicted state trajectory. |
| `getHorizon()` | Returns the prediction horizon length. |

###  `PID` Class
| Function | Description |
|---------|-------------|
| `PID::PID(...)` | Initializes the PID controller. |
| `update(error)` | Computes PID control output from error input. |

###  `Controller` Class
| Function | Description |
|---------|-------------|
| `Controller::Controller()` | Sets up ROS, loads waypoints, builds trajectory. |
| `startControlLoop()` | Main loop for state estimation, MPC, PID, and actuation. |
| `stateEstimatorCallback(...)` | Updates robot state from odometry. |
| `selectNearestPoint(...)` | Selects the closest trajectory point. |
| `interpolateVelocities(...)` | Smooths MPC outputs for high-rate PID. |
| `update_wheel_speeds(...)` | Computes and sends wheel commands. |
| `generateTrajectory(...)` | Generates timed trajectory from waypoints. |
| `interpolateTrajectory(...)` | Resamples trajectory for MPC. |
| `visualizePredictedTrajectory(...)` | Publishes markers for MPC path in RViz. |
| `logStates(...)` | Stores current state for logging. |
| `saveLogFile()` | Writes log data to CSV. |
| `readWaypoints(...)` | Loads and validates waypoints from CSV. |
| `writeTrajectoryToCSV(...)` | Exports resampled trajectory. |

---




## Build and Run

```bash
# Clone the repository
cd ~/catkin_ws/src
git clone https://github.com/your-username/Mobile_Robot_Base_.git
cd ~/catkin_ws
catkin_make

# Source workspace
source devel/setup.bash

# Run the controller
rosrun motion_control motion_control_start
```

>  Ensure that ROS is installed and the robot simulation or hardware is available.

---

##  Future Improvements

- Add dynamic obstacle avoidance using MPC constraints.
- Integrate adaptive PID tuning based on model mismatch.
- Expand to support omnidirectional/mecanum base models.

---

##  Visualization

The predicted MPC trajectory and current reference path are published to RViz as MarkerArrays for real-time visualization.

---

##  Contact

Feel free to reach out for questions, improvements, or collaborations!

**Berke Erikan**  
 [GitHub](https://github.com/berkeerikan) | [LinkedIn](https://www.linkedin.com/in/berkeerikan)
