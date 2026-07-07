# Development Log

This document summarizes the weekly progress during the internship.

---
## Booster in Webots (week 1)

### Goal

Run the Booster robot inside Webots using the Booster SDK.
- webots documentation: https://cyberbotics.com/doc/guide/index
- Example: Development Based on Webots Simulation: https://booster.feishu.cn/wiki/XAS3wv4lwiSiXXkDbMrceE6UnHc#doxcnZGLz6aQpFxiVef76g8W4Xg

### Problems solved

- lots of error messages when loading the webots booster example, no background because the proto files were missing → Using other protos.
- run file does not connect to webots because webots is running through snap → Using anothoer webots version through /usr/local/webots/webots.
- movement shows error messages because the IP is wrong → All on one host, so 127.0.0.1 can be used.

### How to use

- open terminal: webots → Webots opens
- open the right world: Fiile > Open World > ..Documents/webots/webots_og-booster/webots_simulation/worlds/T1_release.wbt → world opens, checkerd floor and booster robot
- open terminal: ~/Documents/webots/webots_og-booster$ ./booster-runner-webots-full-0.0.11.run → controller service runs forever and listens to potential commands
- open terminal: ~/Documents/webots/webots_og-booster/booster_robotics_sdk/build$ ./b1_loco_example_client 127.0.0.1 → subscriber listens to commands.
    - mw enter → ready for movement commands
    - m enter → forward
    - a enter → left
    - s enter → backwards
    - l enter → stop moving

## Using Gazebo (week 2)

### Goal

Connect Gazebo with ROS 2.
- ROS 2 installed
- Install Gazebo https://gazebosim.org/docs/fortress/install_ubuntu/
  - test with another Gazebo version: https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html
- `source /opt/ros/humble/setup.bash`
- `sudo apt install ros-humble-gazebo-ros-pkgs`
- test with the correct Gazebo version: https://classic.gazebosim.org/tutorials?tut=build_world

### Problems solved

- Some Gazebo versions are not compatible because Gazebo has different releases, such as **Gazebo Classic** and **Gazebo Ignition**. Use the correct version. In this case, use **Gazebo Classic**.

### How to use

- `ros2 launch gazebo_ros gazebo.launch.py`
- Or start Gazebo without ROS 2: `gazebo`

## Isaac Sim with ROS 2 (week 2)

### Goal

- Install Isaac Sim: https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/quick-install.html
- Connect Isaac Sim with ROS 2: https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.html#isaac-ros-workspace
- Launch Isaac Sim using ROS 2: https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_launch.html

### Problems solved

- Isaac Sim could not start because of GPU-related issues caused by the software hanging. Restarting the system with `sudo reboot` resolved the problem.
- Isaac Sim could only be launched locally, not from any directory. The solution was to place the package inside `r2_ws/src` and create a ROS 2 wrapper package.

### How to use

- Launch Isaac Sim using ROS 2:
`ros2 launch isaacsim_bringup run_isaacsim.launch.py`
- Or launch Isaac Sim directly without ROS 2:
`~/isaacsim/isaac-sim.sh`

## Isaac Lab (week 3)

### Goal

- Install Isaac Lab on top of Isaac Sim.
- Verify that Isaac Lab runs correctly.

### Problems solved

- Isaac Lab could not be installed because Isaac Sim was not installed correctly. Reinstalling Isaac Sim with `pip` inside a virtual environment solved the issue.
- Installation failed because several package versions were too new. Downgrading the required packages resolved the dependency conflicts.
- Installing Booster together with Isaac Sim and Isaac Lab in the same environment caused conflicts. The solution was to reinstall everything using the Booster installation procedure.

### How to use

- `wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh bash Miniconda3-latest-Linux-x86_64.sh` (Conda was later removed because it was no longer needed.)
- ROS 2: `source /opt/ros/humble/setup.bash`
- Launch Isaac Lab: `conda activate env_isaaclab` `cd ~/IsaacLab ./isaaclab.sh -p scripts/tutorials/00_sim/create_empty.py`

## Booster SDK in Isaac Sim (week 3)

### Goal

- Run the Booster SDK together with Isaac Sim.
- Test communication between the Booster SDK and the simulator.

### Problems solved

- The Booster SDK could not be installed because the required Python version was too old. Isaac Sim provides its own Python interpreter, which was used instead.
- The Booster robot could not be controlled because importing the USD file alone is not sufficient, and the `package.run` file was incompatible with the installed Isaac Sim version.
- The `package.run` file could not be executed because Isaac Sim 5.1 uses Python 3.11 while ROS 2 Humble uses Python 3.10. The solution was to uninstall Isaac Sim 5.1 and use Isaac Sim 4.2 instead.

### How to use

- Start Isaac Sim: `./IsaacSim/_build/linux-x86_64/release/isaac-sim.sh`
- Python executable: `./_build/linux-x86_64/release/python.sh`
- Booster SDK example: `conda activate booster_sdk ./booster_robotics_sdk/build/b1_loco_example_client enp7s0`
  Example output: `ChannelSubscriber::InitChannel: setting reliability: 1 queue_capacity: 1024`
- Football example: `~/Workspace/tools/isaac_package_0.0.6.run \ ~/IsaacSim/_build/linux-x86_64/release/python.sh`
  This example does not work because it attempts to start ROS 2, which requires a different Python version.
  - Alternative: `~/Workspace/tools/booster-runner-full-0.0.10.run`
  
## Run the Booster Robot in Isaac Sim (week 4 - 6)

### Goal

- Run the Booster robot inside Isaac Sim.
- Connect the Booster SDK with the simulator.
- Enable robot movement inside the simulated environment.

### Problems solved

- Installation
  - The `package.run` installer expects Isaac Sim to be installed through Omniverse Launcher. Creating the expected installation path with a symbolic link solved this issue.
    `mkdir -p ~/.local/share/ov/pkg/ ln -s ~/isaacsim ~/.local/share/ov/pkg/isaac-sim-4.2.0`
  - The Booster Runner could not start because the configuration files were missing. Extracting the installer and creating the required configuration files solved the problem.
- GPU and Driver Issues
  - Isaac Sim froze because the physics simulation started before the robot was fully initialized.
  - Isaac Sim displayed a black screen because CUDA was not working correctly.
  - The laptop contains an Intel iGPU and an NVIDIA RTX 5060 Mobile GPU, which requires NVIDIA Open Kernel Modules.
  - Installing `nvidia-driver-580-open` solved the CUDA initialization problem and enabled `nvidia-smi` to detect the GPU correctly.
- Robot and Physics
  - The robot model originally contained 36 DOFs while the Booster robot has only 27 DOFs. The simulation code had to be modified to generate the correct physics automatically.
  - PhysX reported articulation errors because the articulation root was missing. Applying `UsdPhysics.ArticulationRootAPI` fixed the problem.
  - Some individual USD files also required manual corrections.
- Display
  - `prime-select on-demand` works for external displays but prevents Isaac Sim from starting.
  - `prime-select nvidia` allows Isaac Sim to start but may freeze the laptop display or the second monitor.
  
### How to use

Install Isaac Sim 4.2
`mkdir ~/isaacsim`
`cd ~/Downloads`

unzip `isaac-sim-standalone*.zip -d ~/isaacsim`

`cd ~/isaacsim`
`./omni.isaac.sim.post.install.sh`

Verify the GPU:

`nvidia-smi`

Accept the warning dialog during the first startup.

Prepare Booster

- Move the Booster run files into the Workspace.
- Use the Isaac Simulation environment.
- Load the world manually after Isaac Sim starts:

File
 └── Add Reference
      └── Documents/isaac/T1_usd/t1.usd

Disable IOMMU.

Enable the ROS 2 bridge:

Window
 └── Extensions
      └── omni.isaac.ros2_bridge

Rebuild the Booster SDK

`cd booster_robotics_sdk`

`rm -rf build`
`mkdir build`

`cd build`
`cmake ..`
`make`

Start Isaac Sim
`~/.local/share/ov/pkg/isaac-sim-4.2.0/isaac-sim.sh`

Start the Booster package
`./Workspace/isaac_package_t1_7dof_arms_hand_0.0.2.run`

Start the Booster Runner
`./booster_stack/booster-runner-full-7dof_arms-0.0.4.run`

Start robot movement
`./booster_robotics_sdk/build/b1_loco_example_client 127.0.0.1`

Available commands:
mp, mw, w, a, s, d, l, e

## ROS 2 Bridge (week 5)

### Goal

- Connect the Booster ROS 2 package with Isaac Sim.
- Verify that ROS 2 communication works correctly.
  
### Problems solved

- `colcon build` failed because of an incompatible `em` installation in the Python environment.
- Reinstalling EmPy (Python Template Engine) resolved the build issue.
  
### How to use

- Enable the ROS 2 bridge in Isaac Sim.
- Build the ROS 2 workspace if required.
- Source the workspace:
  `source install/setup.bash`
- Run the Booster ROS 2 example:
  `cd ~/booster_robotics_sdk_ros2/booster_ros2_example`
  `ros2 run booster_rpc_client rpc_client_node`
- If everything is configured correctly, the node should connect successfully and wait for communication.

## Optimize Isaac Booster Performance (week 6 - 8)

### Goal

- Improve the startup time and overall performance of the Booster simulation.
- Create a lightweight Booster environment package.

### Problems solved

- The simulation remained relatively slow. Performance is expected to improve on more powerful hardware.
  
### How to use

- Create a custom Booster environment package.
- Remove unnecessary CSV files.
- Reduce the `frame_count` value.
- Treat `start_ros2_local_isaac_sim.sh` in `package_extract` the same way as the original package launcher.

## Design an Isaac Simulation Environment (week 7 - 9)

### Goal

- Create a custom simulation environment for the Booster robot.
- Add obstacles and objects for RoboCup-style scenarios.

### Problems solved

- The robot continued moving after a collision even though the Python script was supposed to stop it. The Booster SDK continued sending motion commands.
- A collision check was added to stop all torque commands when a collision is detected.
  `if self._collision_detected: self._ks_robot._ros_torque_command = np.zeros( len(self._ks_robot._ros_torque_command) ) return`
- Initialize the collision flag: `self._collision_detected = False`
- This alone did not solve the issue because the finger physics were incorrect. Switching to a different robot model resolved the problem.

### How to use

- Follow the NVIDIA Isaac Sim environment tutorial.
- Modify the simulation world:
package_extract/
└── booster_sim/
    └── booster_isaac/
        └── booster_standalone_ros2_robocup_t1_7dof_arms_hand.py
- Edit scene/default.usd.
- Add walls and obstacles.
- For every static object:
  - Add → Physics → Rigid Body with Colliders Preset
- Add RoboCup objects such as shelves, labyrinths, or other indoor obstacles.

## Improvements (week 9+)

### Goal

- Upgrade the project from Isaac Sim 4.2 to Isaac Sim 5.1.
- Improve the Booster hand simulation.
- Keep compatibility with both Isaac Sim versions.

### Problems solved

- Isaac Sim 5.1 renamed the Python modules from:
  `omni.isaac.*` to `isaacsim.*`
- The project code had to be updated accordingly.
- The camera image appeared blurry. Setting the camera fstop from 2.0 to 0 `fixed` the issue.
- Isaac Sim 5.1 uses Python 3.11, while Isaac Sim 4.2 uses Python 3.10. ROS 2 therefore requires a different setup for each version.

### How to use

- Configure ROS 2 for Isaac Sim 5.1
  - `source /opt/ros/humble/setup.bash`
  -  `sudo apt install \`
     `python3-catkin-pkg \`
     `python3-packaging \`
     `python3-colcon-common-extensions`
  - `cd /home/ltl7/package_extract_51/booster_ros2`
  - `rm -rf build install log`
  - `colcon build --symlink-install`
    
- Improve the Booster Hand
  - Disable collision for the fingers.
  - Keep collision enabled for the hand link base.
    
- Run Isaac Sim 5.1
  - `source /opt/ros/humble/setup.bash`
  -  `source /home/ltl7/package_extract_51/booster_ros2/install/setup.bash`
  - Start Isaac Sim: `./isaacsim/5.1/isaac-sim-5.1.sh`
  - Start the Booster environment: `./package_extract_51/start_ros2_local_isaac_sim.sh`
  - Start the Booster Runner: `./booster_stack/booster-runner-full-7dof_arms-0.0.4.run`
  - Start the SDK client: `./booster_robotics_sdk/build/b1_loco_example_client 127.0.0.1`
  - Movement commands: mp, mw, w, a, s, d, l, e
    
- Run Isaac Sim 4.2
  - Start Isaac Sim: `~/.local/share/ov/pkg/isaac-sim-4.2.0/isaac-sim.sh`
  - Start the Booster environment: `./package_extract_42/start_ros2_local_isaac_sim.sh`
  - Start the Booster Runner: `./booster_stack/booster-runner-full-7dof_arms-0.0.4.run`
  - Start the SDK client: `./booster_robotics_sdk/build/b1_loco_example_client 127.0.0.1`
  - Movement commands: mp, mw, w, a, s, d, l, e
    
## Final Result

The final system provides

- Booster robot simulation in Isaac Sim 5.1
- ROS 2 integration
- RoboCup@Home environment
- Camera support
- Existing Booster policy support
- Improved startup performance
- Collision-aware robot control
