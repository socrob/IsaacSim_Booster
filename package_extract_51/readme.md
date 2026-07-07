## build msgs
cd booster_ros2
colcon build

## packageing
makeself ../isaac_package_t1_7dof_arms_hand ../isaac_package_t1_7dof_arms_hand_0.0.2.run "isaac package" ./start_ros2_local_isaac_sim.sh 

## run
./start_ros2_local_isaac_sim.sh 