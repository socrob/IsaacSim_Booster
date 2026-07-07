# IsaacSim Booster
## Controllable Booster Robot in Isaac Sim 5.1 with RoboCup@Home Environment

## Overview

This repository contains the development of a simulation environment for the Booster humanoid robot using **NVIDIA Isaac Sim 5.1**.

The project was completed during an internship with the goal of creating a simulation framework that allows testing robot functionality before deployment on the physical robot.

The simulation includes

- Booster humanoid robot
- ROS 2 integration
- Domestic RoboCup@Home environment
- Camera sensor support
- Collision handling
- Support for existing Booster control policies

## Running the Simulation
- Download `booster-runner-full-7dof_arms-0.0.4.run` form https://booster.feishu.cn/wiki/XAS3wv4lwiSiXXkDbMrceE6UnHc#doxcndlVE80Kz5m6KSwh06GBexg at save it in the folder `./booster_stack`
```bash
source /opt/ros/humble/setup.bash
source package_extract_51/booster_ros2/install/setup.bash
```

### Start the Booster Environment

```bash
./package_extract_51/start_ros2_local_isaac_sim.sh
```

### Start the Booster Runner

```bash
./booster_stack/booster-runner-full-7dof_arms-0.0.4.run
```

### Control the Robot

```bash
./booster_robotics_sdk/build/b1_loco_example_client 127.0.0.1
```
Example commands

- `mw` – ready for walking
- `w` – forward
- `s` – backward
- `a` – left
- `e` – right
- `l` – stop

## RoboCup@Home Environment

The environment contains a simplified domestic scene including

- walls
- shelves
- indoor obstacles
- navigation space

allowing the Booster robot to perform navigation and interaction tasks in a realistic home environment.

## Known Issues

- Performance depends heavily on GPU hardware.
- The environment is intended for research and development purposes.

## Development Log

A complete week-by-week development history can be found in

**DEVELOPMENT_LOG.md**
