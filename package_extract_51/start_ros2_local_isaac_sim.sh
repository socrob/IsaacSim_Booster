#!/bin/bash

# 获取脚本所在目录的绝对路径
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 默认路径
default_isaac_path="/home/$USER/isaacsim/5.1/python.sh"

isaac_path=$default_isaac_path


asset_path=""

# 遍历所有传递的参数
for arg in "$@"; do
  if [[ $arg == *"python.sh"* ]]; then
    isaac_path=$arg
    echo "Isaac path provided: $isaac_path"
  elif [[ $arg == *".usd"* ]]; then
    asset_path=$arg
    echo "Asset path provided: $asset_path"
  fi
done

export FASTRTPS_DEFAULT_PROFILES_FILE=$SCRIPT_DIR/fastdds_profile.xml

# 设置ROS消息包的路径
export BOOSTER_ROS2=$SCRIPT_DIR/booster_ros2

# 确保安装目录存在
if [ ! -d "$BOOSTER_ROS2/install" ]; then
    echo "The build time path '$BOOSTER_ROS2/install' doesn't exist. Please build the ROS2 workspace first."
    exit 1
fi

# 设置COLCON_CURRENT_PREFIX环境变量
export COLCON_CURRENT_PREFIX=$BOOSTER_ROS2/install

# Source ROS2 workspace
source $BOOSTER_ROS2/install/setup.sh

killall robot_state_publisher
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro ./urdf/V23RobotAsmSerial.urdf)" &


# 启动Isaac Sim
if [ -z "$asset_path" ]; then
    $isaac_path $SCRIPT_DIR/booster_sim/booster_isaac/booster_standalone_ros2_robocup_t1_7dof_arms_hand.py
else
    $isaac_path $SCRIPT_DIR/booster_sim/booster_isaac/booster_standalone_ros2_robocup_t1_7dof_arms_hand.py --asset_path $asset_path
fi