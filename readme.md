# Quadcopter ROS2 Navio Project

## Overview
This project involves building a quadcopter controlled using ROS2 with various components for sensor integration, control, and manual operation. The physical setup includes a Raspberry Pi, Navio2 board, and necessary motors and sensors, while the software comprises multiple nodes communicating to achieve autonomous flight control and manual teleoperation.

## Repository Tree Structure
```
quadcopter_project/
├── src/
│   ├── imu_node/
│   │   ├── imu_node.cpp
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── gps_node/
│   │   ├── gps_node.cpp
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── battery_monitor_node/
│   │   ├── battery_monitor_node.cpp
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── state_estimator_node/
│   │   ├── state_estimator_node.cpp
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── controller_node/
│   │   ├── controller_node.cpp
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── mission_planner_node/
│   │   ├── mission_planner_node.cpp
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── motor_control_node/
│   │   ├── motor_control_node.cpp
│   │   ├── CMakeLists.txt
│   │   └── package.xml
├── msg/
│   ├── MotorControl.msg
│   └── StateEstimate.msg
├── launch/
│   └── quadcopter_launch.py
├── scripts/
│   ├── calibration.py
│   └── teleop.py
└── README.md
```

## Physical Components
- **Raspberry Pi 4**: The main computing platform running Ubuntu with ROS2.
- **Navio2 Board**: An expansion board for the Raspberry Pi that provides sensors like IMU, GPS, and PWM outputs for motor control.
- **Chassis, Motors, Propellers**: The physical body of the quadcopter with four brushless motors and propellers for flight.
- **Battery**: Provides power to the motors and electronics.

## Node Descriptions
### 1. IMU Node (`imu_node`)
- **Purpose**: Reads IMU data (e.g., accelerometer and gyroscope) from the Navio2 board.
- **Role**: Publishes IMU data to be used by the State Estimator node.

### 2. GPS Node (`gps_node`)
- **Purpose**: Retrieves GPS data from the Navio2 board.
- **Role**: Provides position information to aid in the state estimation.

### 3. Battery Monitor Node (`battery_monitor_node`)
- **Purpose**: Monitors battery voltage and current.
- **Role**: Publishes battery information to ensure safe operation.

### 4. State Estimator Node (`state_estimator_node`)
- **Purpose**: Estimates the current state (position, velocity) of the quadcopter based on sensor inputs (IMU, GPS).
- **Role**: Provides state estimates used by the Controller node for control decisions.

### 5. Controller Node (`controller_node`)
- **Purpose**: Computes control commands based on the current state and desired trajectory.
- **Role**: Publishes `MotorControl` messages to direct the Motor Control node on how to adjust motor speeds.
- **Note**: LQR-based control is used, and gains are precomputed and defined globally (analysis and optimization TBD, placeholder values for now).

### 6. Mission Planner Node (`mission_planner_node`)
- **Purpose**: Plans the quadcopter’s mission (e.g., waypoint following).
- **Role**: Publishes high-level commands to the Controller node.

### 7. Motor Control Node (`motor_control_node`)
- **Purpose**: Listens to `MotorControl` messages from the Controller node and translates them into PWM signals for motor control.
- **Role**: Physically actuates the motors to achieve the desired thrust, roll, pitch, and yaw.

## Launch File (`quadcopter_launch.py`)
The `quadcopter_launch.py` launch file is used to bring up all the nodes required for the quadcopter operation. This includes the following nodes:
- IMU Node
- GPS Node
- Battery Monitor Node
- State Estimator Node
- Controller Node
- Mission Planner Node
- Motor Control Node

This ensures that all nodes start together, enabling the quadcopter to process sensor data, make control decisions, and execute movements seamlessly.

## Scripts
### Calibration Script (`calibration.py`)
- **Purpose**: Calibrates the IMU sensors by collecting a specified number of samples and calculating biases for accelerometer and gyroscope data.
- **How It Works**: The script gathers IMU data, averages the samples to determine bias, and logs the results. The calibrated bias values help in improving sensor accuracy by correcting systematic offsets.

### Teleoperation Script (`teleop.py`)
- **Purpose**: Provides manual control of the quadcopter using the keyboard.
- **How It Works**: The user can use `WASD` keys for directional movement (forward, backward, left, right) and `Q/E` for yaw control. The script reads keyboard inputs and publishes corresponding `Twist` messages to the command topic, allowing manual intervention in quadcopter operation.

## Usage
1. **Setup**: Make sure all the physical components are properly connected (Navio2 on Raspberry Pi, motors, battery).
2. **Calibration**: Run `calibration.py` to calibrate the IMU sensors before flight.
3. **Launch the System**: Use `quadcopter_launch.py` to start all necessary nodes.
   ```bash
   ros2 launch quadcopter_launch.py
   ```
4. **Manual Control**: Optionally run `teleop.py` to manually control the quadcopter using keyboard commands.

## Future Improvements
- **PID Control**: Add a PID control option to the controller node for simpler tuning.
- **Advanced State Estimation**: Implement a Kalman Filter for more robust state estimation.
- **Autonomous Missions**: Expand the mission planner node to support complex autonomous missions.

Feel free to contribute to further improve the project by adding more advanced features or refining the current control strategies!

