# AMR Controller

An Autonomous Mobile Robot (AMR) controller developed for mapping and navigation applications. This project implements a complete ROS2-based system utilizing SLAM toolbox for simultaneous localization and mapping, and Nav2 stack for autonomous navigation.

**Course Module:** EN2160 - Electronic and Telecommunication Department, University of Moratuwa, Sri Lanka

---

## Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Hardware Design](#hardware-design)
  - [PCB Design](#pcb-design)
  - [Mechanical Design](#mechanical-design)
- [Software Stack](#software-stack)
- [Implementation](#implementation)
- [Simulation Results](#simulation-results)
- [Repository Structure](#repository-structure)
- [Hardware Constraints](#hardware-constraints)
- [Installation](#installation)
- [Usage](#usage)
- [References](#references)

---

## Overview

This project presents a comprehensive AMR controller designed to provide plug-and-play functionality for autonomous mobile robots equipped with LiDAR sensors. The controller integrates advanced navigation algorithms and hardware interfaces to enable autonomous operation in complex environments.

### Key Features

- **SLAM Capabilities:** Implemented using slam_toolbox for real-time mapping and localization
- **Autonomous Navigation:** Nav2 stack for path planning and obstacle avoidance
- **ROS2 Framework:** Built on ROS2 Humble distribution
- **Differential Drive Control:** Custom ros2_control hardware interface
- **Modular Design:** Hardware and software components designed for flexibility and scalability
- **Simulation Environment:** Full Gazebo simulation support with URDF/Xacro robot descriptions

---

## System Architecture

### Block Diagram

![System Block Diagram](docs/design_methdology/pics/_page_18_Picture_1.jpeg)

The system architecture consists of:

- **Laptop/Processing Unit:** Runs simulation environment and high-level control algorithms
- **Arduino Uno:** Intermediary controller for motor driver interface
- **Motor Driver:** Custom PCB based on MC33886 IC for dual motor control
- **Motors:** Differential drive configuration
- **Communication:** UART protocol between processing unit and Arduino
- **Power Supply:** Dedicated power management for motors and electronics

---

## Hardware Design

### PCB Design

The custom motor driver PCB was designed to control two DC motors independently, essential for differential drive operation. The design utilizes the MC33886 H-bridge motor driver IC, selected for its reliability and protection features.

#### Motor Driver Block Diagram

![Motor Driver Block Diagram](docs/design_methdology/pics/_page_20_Picture_5.jpeg)

**Key Features:**
- Dual MC33886 IC configuration for independent motor control
- PWM-based speed control
- Direction control through binary signals
- Built-in protection features (over-current, thermal protection)
- Compact PCB layout optimized for mobile robot integration

#### PCB Views

![Routed PCB Design](docs/design_document/pics/_page_7_Picture_5.jpeg)

![Bare PCB Front View](docs/design_document/pics/_page_9_Picture_0.jpeg)

![Bare PCB Rear View](docs/design_document/pics/_page_9_Picture_2.jpeg)

![Soldered PCB](docs/design_document/pics/_page_9_Picture_4.jpeg)

**Component Selection:**
- **ICs:** MC33886 H-bridge motor drivers
- **Passive Components:** SMD resistors, capacitors for filtering and decoupling
- **Connectors:** Headers for Arduino interface and motor terminals
- **Power Management:** Voltage regulation and distribution circuits

### Mechanical Design

The mechanical platform provides a stable base for the AMR system with considerations for component mounting, weight distribution, and maneuverability.

#### Platform Design

![Mechanical Sketch](docs/design_methdology/pics/_page_24_Picture_5.jpeg)

![Component Layout](docs/design_methdology/pics/_page_25_Picture_6.jpeg)

**Design Specifications:**
- Four-wheeled configuration: Two motor-driven wheels (rear) and two caster wheels (front)
- Differential drive kinematics for omnidirectional movement
- Mounting provisions for electronics, battery, and sensors
- Industrial caster wheels for stability and load support
- Direct motor integration for efficient power transmission

#### Motor Driver Enclosure

![Enclosure Design](docs/design_document/pics/_page_20_Figure_0.jpeg)

![Enclosure Assembly](docs/design_document/pics/_page_21_Figure_0.jpeg)

The enclosure design protects the PCB while providing thermal management and accessibility for connections.

#### 3D Printed Components

![Mold Design](docs/design_methdology/pics/_page_44_Picture_1.jpeg)

![Enclosure Bottom Part](docs/design_methdology/pics/_page_37_Picture_1.jpeg)

![Enclosure Assembly](docs/design_methdology/pics/_page_37_Picture_4.jpeg)

---

## Software Stack

### Technology Stack

- **ROS2 Humble:** Core framework for robot software architecture
- **slam_toolbox:** SLAM implementation for mapping and localization
- **Nav2:** Navigation stack for path planning and execution
- **Gazebo:** Physics simulation environment
- **RViz2:** 3D visualization tool
- **ros2_control:** Real-time control framework
- **URDF/Xacro:** Robot description format

### Software Architecture

![Software Block Diagram](docs/design_methdology/pics/_page_28_Figure_5.jpeg)

![Software Flowchart](docs/design_methdology/pics/_page_29_Figure_6.jpeg)

**Key Components:**

1. **Robot Model (URDF/Xacro)**
   - Complete robot description with joints, links, and sensors
   - LiDAR sensor integration
   - Differential drive plugin configuration

2. **Hardware Interface (diffdrive_arduino)**
   - Custom ros2_control hardware interface
   - UART communication with Arduino
   - Motor encoder feedback processing
   - Real-time odometry computation

3. **Control System**
   - diff_drive_controller for velocity commands
   - twist_mux for command prioritization
   - Keyboard teleoperation support

4. **SLAM Pipeline**
   - Real-time map generation using LiDAR data
   - Pose estimation and correction
   - Map frame publication for localization accuracy

5. **Navigation System**
   - Global and local path planning
   - Costmap generation for obstacle representation
   - Dynamic obstacle avoidance
   - Behavior tree execution for navigation tasks

---

## Implementation

### Simulation Environment

![Gazebo Simulation World](docs/design_methdology/pics/_page_31_Picture_3.jpeg)

![Gazebo Simulation](docs/design_document/pics/_page_129_Figure_4.jpeg)

The robot model was developed and tested in Gazebo simulation before hardware implementation. The simulation environment includes:

- Custom world models for testing
- Accurate physics parameters
- Sensor simulation (LiDAR, encoders)
- Real-time visualization in RViz2

### Hardware Interface

The hardware interface connects the simulated/planned motion commands to physical motor control:

```
High-Level Commands (Nav2) → twist_mux → diff_drive_controller → 
diffdrive_arduino → Arduino → Motor Driver → Motors
```

**Communication Flow:**
1. Nav2 publishes velocity commands (cmd_vel)
2. diff_drive_controller converts to wheel velocities
3. diffdrive_arduino sends commands via UART
4. Arduino generates PWM signals for motor driver
5. Encoder feedback returns to odometry computation

---

## Simulation Results

### SLAM Mapping

The system was tested in two different simulated environments to validate SLAM performance.

#### Room 1 Mapping

![Room 1 Gazebo Environment](docs/design_document/pics/_page_124_Picture_5.jpeg)

![Room 1 RViz Map](docs/design_document/pics/_page_125_Picture_0.jpeg)

**Environment Characteristics:**
- Rectangular room with obstacles
- Static furniture and walls
- Clear corridors for navigation

#### Room 2 Mapping

![Room 2 Gazebo Environment](docs/design_document/pics/_page_126_Picture_0.jpeg)

![Room 2 RViz Map](docs/design_document/pics/_page_126_Picture_2.jpeg)

**Environment Characteristics:**
- More complex layout
- Multiple rooms and doorways
- Varied obstacle configurations

### Navigation Performance

![Navigation While Mapping](docs/design_document/pics/_page_132_Picture_0.jpeg)

![2D Goal Selection for Navigation](docs/design_document/pics/_page_135_Picture_0.jpeg)

The Nav2 stack successfully demonstrated:
- Autonomous goal-seeking behavior
- Dynamic path replanning
- Obstacle avoidance
- Smooth trajectory execution

### TF Frame Structure

![TF Tree](docs/design_document/pics/_page_130_Figure_0.jpeg)

The coordinate frame hierarchy showing relationships between:
- `map` frame (global reference)
- `odom` frame (odometry-based localization)
- `base_link` frame (robot center)
- `laser` frame (LiDAR sensor)

---

## Repository Structure

```
amr_ws/
├── src/                          # ROS2 source packages
│   ├── my_bot/                   # Main robot package
│   │   ├── description/          # URDF/Xacro robot models
│   │   ├── launch/               # Launch files
│   │   ├── config/               # Configuration files
│   │   └── worlds/               # Gazebo world files
│   ├── diffdrive_arduino/        # Hardware interface package
│   ├── serial/                   # Serial communication library
│   ├── tele/                     # Teleoperation package
│   └── twist_mux/                # Command multiplexer
├── motor_driver_pcb_schematic/   # PCB design files
│   ├── motor_driver.GBL          # Gerber files
│   ├── motor_driver.EXTREP       # Design reports
│   └── *.apr                     # PCB project files
├── docs/                         # Documentation
│   ├── design_document/          # Detailed design document
│   │   ├── AMR_Controller_Design_Document.md
│   │   └── pics/                 # Design images
│   └── design_methdology/        # Methodology document
│       ├── AMR_controller_Design_Methodology.md
│       └── pics/                 # Methodology images
├── robot.sh                      # Launch script
└── README.md                     # This file
```

---

## Hardware Constraints

This project was developed under certain hardware constraints that influenced the final implementation:

1. **Processing Unit:** Jetson Nano was unavailable; a laptop was used for simulation and high-level processing
2. **Sensors:** Commercial LiDAR sensor was not available during initial development phase
3. **Encoders:** Proper motor encoders were not available; odometry relies on motor driver feedback
4. **Testing Platform:** Real-time hardware-in-the-loop testing conducted with Arduino-based interface

These constraints led to a hybrid approach where simulation validates the software stack, while hardware implementation focuses on motor control and basic odometry.

---

## Installation

### Prerequisites

- Ubuntu 22.04 LTS
- ROS2 Humble Hawksbill
- Gazebo 11
- Python 3.10+

### Dependencies

```bash
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-twist-mux
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers
sudo apt install ros-humble-gazebo-ros2-control
```

### Build Instructions

```bash
# Clone the repository
cd ~/
git clone <repository-url> amr_ws
cd amr_ws

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash
```

---

## Usage

### Simulation

Launch the complete simulation environment with Gazebo and RViz:

```bash
# Terminal 1: Launch robot in Gazebo
ros2 launch my_bot launch_sim.launch.py

# Terminal 2: Launch SLAM
ros2 launch slam_toolbox online_async_launch.py

# Terminal 3: Launch Navigation
ros2 launch nav2_bringup navigation_launch.py

# Terminal 4: Teleoperation (optional)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Hardware Control

```bash
# Launch hardware interface
ros2 launch my_bot hardware_launch.py
```

### Mapping

To create a new map:

```bash
# Start SLAM and drive the robot
ros2 launch slam_toolbox online_async_launch.py

# Save the map
ros2 run nav2_map_server map_saver_cli -f <map_name>
```

---

## References

### Documentation

For detailed information, refer to:
- [Design Document](docs/design_document/AMR_Controller_Design_Document.md)
- [Design Methodology](docs/design_methdology/AMR_controller_Design_Methodology.md)

### Key Technologies

- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Nav2 Documentation](https://navigation.ros.org/)
- [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Gazebo](http://gazebosim.org/)
- [ros2_control](https://control.ros.org/)

### Component Datasheets

- MC33886 H-Bridge Motor Driver IC
- Arduino Nano specifications
- LiDAR sensor specifications (when integrated)

---