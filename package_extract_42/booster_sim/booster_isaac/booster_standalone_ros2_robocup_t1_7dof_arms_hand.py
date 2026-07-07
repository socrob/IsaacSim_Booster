# Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#


"""
Introduction
We start a runner which publishes robot sensor data as ROS2 topics and listens to outside ROS2 topic
"""

from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})
import os
import carb
import time
import math
import argparse
import threading
import numpy as np
import omni.appwindow  # Contains handle to keyboard
import omni.graph.core as og
import omni.usd
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.sensor import IMUSensor, ContactSensor
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core.utils.prims import define_prim, get_prim_at_path
from pxr import UsdPhysics, Usd, PhysxSchema, Gf

from datetime import datetime

# enable ROS2 bridge extension
ext_manager = omni.kit.app.get_app().get_extension_manager()
ext_manager.set_extension_enabled_immediate("omni.isaac.ros2_bridge", True)

# check if rosmaster node is running
# this is to prevent this sample from waiting indefinetly if roscore is not running
# can be removed in regular usage

# import ROS2 python bindings
import rclpy
from rclpy.node import Node
# ros-python and ROS1 messages
import geometry_msgs.msg as geometry_msgs
import sensor_msgs.msg as sensor_msgs
import booster_msgs.msg as booster_msgs
from builtin_interfaces.msg import Time as BuiltinTime
from booster_msgs.msg import HandDdsMsg
from scipy.interpolate import CubicSpline

# isaac
# sim_joint_names = ['AAHead_yaw', 'Left_Shoulder_Pitch', 'Right_Shoulder_Pitch', 'Waist', 
#                     'Head_pitch', 'Left_Shoulder_Roll', 'Right_Shoulder_Roll', 'Left_Hip_Pitch', 
#                     'Right_Hip_Pitch', 'Left_Elbow_Pitch', 'Right_Elbow_Pitch', 'Left_Hip_Roll',
#                     'Right_Hip_Roll', 'Left_Elbow_Yaw', 'Right_Elbow_Yaw', 'Left_Hip_Yaw',
#                     'Right_Hip_Yaw', 'Left_Wrist_Pitch', 'Right_Wrist_Pitch', 'Left_Knee_Pitch', 
#                     'Right_Knee_Pitch', 'Left_Wrist_Yaw', 'Right_Wrist_Yaw', 'Left_Ankle_Pitch', 
#                     'Right_Ankle_Pitch', 'Left_Hand_Roll', 'Right_Hand_Roll', 'Left_Ankle_Roll', 
#                     'Right_Ankle_Roll', 'left_Link1', 'left_Link2', 'right_Link1', 'right_Link2',
#                     'left_Link11', 'left_Link22', 'right_Link11', 'right_Link22']


sim_joint_names = ['AAHead_yaw', 'Left_Shoulder_Pitch', 'Right_Shoulder_Pitch', 'Waist', 
                   'Head_pitch', 'Left_Shoulder_Roll', 'Right_Shoulder_Roll', 'Left_Hip_Pitch', 
                   'Right_Hip_Pitch', 'Left_Elbow_Pitch', 'Right_Elbow_Pitch', 'Left_Hip_Roll', 
                   'Right_Hip_Roll', 'Left_Elbow_Yaw', 'Right_Elbow_Yaw', 'Left_Hip_Yaw', 
                   'Right_Hip_Yaw', 'Left_Wrist_Pitch', 'Right_Wrist_Pitch', 'Left_Knee_Pitch', 
                   'Right_Knee_Pitch', 'Left_Wrist_Yaw', 'Right_Wrist_Yaw', 'Left_Ankle_Pitch', 
                   'Right_Ankle_Pitch', 'Left_Hand_Roll', 'Right_Hand_Roll', 'Left_Ankle_Roll', 
                   'Right_Ankle_Roll', 
                   'L_index_MCP_joint', 'L_middle_MCP_joint', 'L_pinky_MCP_joint', 'L_ring_MCP_joint', 'L_thumb_MCP_joint1', 
                   'R_index_MCP_joint', 'R_middle_MCP_joint', 'R_pinky_MCP_joint', 'R_ring_MCP_joint', 'R_thumb_MCP_joint1', 
                   'L_index_DIP_joint', 'L_middle_DIP_joint', 'L_pinky_DIP_joint', 'L_ring_DIP_joint', 'L_thumb_MCP_joint2', 
                   'R_index_DIP_joint', 'R_middle_DIP_joint', 'R_pinky_DIP_joint', 'R_ring_DIP_joint', 'R_thumb_MCP_joint2', 
                   'L_thumb_PIP_joint', 'R_thumb_PIP_joint', 
                   'L_thumb_DIP_joint', 'R_thumb_DIP_joint']


default_joint_names = [
               

                "AAHead_yaw", "Head_pitch",
                "Left_Shoulder_Pitch", "Left_Shoulder_Roll", "Left_Elbow_Pitch", "Left_Elbow_Yaw", "Left_Wrist_Pitch", "Left_Wrist_Yaw", "Left_Hand_Roll",
                "Right_Shoulder_Pitch", "Right_Shoulder_Roll", "Right_Elbow_Pitch", "Right_Elbow_Yaw", "Right_Wrist_Pitch", "Right_Wrist_Yaw", "Right_Hand_Roll",
                "Waist",

                "Left_Hip_Pitch", "Left_Hip_Roll", "Left_Hip_Yaw",
                "Left_Knee_Pitch", "Left_Ankle_Pitch", "Left_Ankle_Roll",
                
                "Right_Hip_Pitch", "Right_Hip_Roll", "Right_Hip_Yaw",
                "Right_Knee_Pitch", "Right_Ankle_Pitch", "Right_Ankle_Roll",
]

# jaw_joint_names = ['left_Link1', 'left_Link2', 'left_Link11', 'left_Link22', 'right_Link1', 'right_Link2', 'right_Link11', 'right_Link22']


jaw_joint_names = ['L_index_MCP_joint', 'L_middle_MCP_joint', 'L_pinky_MCP_joint', 'L_ring_MCP_joint', 'L_thumb_MCP_joint1', 
                   'R_index_MCP_joint', 'R_middle_MCP_joint', 'R_pinky_MCP_joint', 'R_ring_MCP_joint', 'R_thumb_MCP_joint1', 
                   'L_index_DIP_joint', 'L_middle_DIP_joint', 'L_pinky_DIP_joint', 'L_ring_DIP_joint', 'L_thumb_MCP_joint2', 
                   'R_index_DIP_joint', 'R_middle_DIP_joint', 'R_pinky_DIP_joint', 'R_ring_DIP_joint', 'R_thumb_MCP_joint2', 
                   'L_thumb_PIP_joint', 'R_thumb_PIP_joint', 
                   'L_thumb_DIP_joint', 'R_thumb_DIP_joint']


def time_to_float(time_msg):
    return time_msg.sec + time_msg.nanosec * 1e-9

ROS_CAMERA_GRAPH_PATH = "/ROS_Camera"


class Booster_direct_runner(Node):
    def __init__(self, physics_dt, render_dt, bg_asset_path) -> None:
        """
        [Summary]

        creates the simulation world with preset physics_dt and render_dt and creates a booster t1 robot inside the world

        Argument:
        physics_dt {float} -- Physics downtime of the scene.
        render_dt {float} -- Render downtime of the scene.

        """
        super().__init__("booster_k2")
        self._world = World(stage_units_in_meters=1.0, physics_dt=physics_dt, rendering_dt=render_dt)
        world = self._world

        current_file_dir = os.path.dirname(os.path.abspath(__file__))

        robot_prim = "/World/robotcup/T1"

        self._physics_dt = physics_dt
        print("Adding background asset: " + bg_asset_path)

        add_reference_to_stage(usd_path = bg_asset_path, 
                               prim_path = "/World/robotcup"
                               )

        #asset_path = current_file_dir + "/scene/t1/t1_with_7dof_arms_hand_1.usd"
        asset_path = current_file_dir + "/scene/t1/t1_with_7dof_arms.usd"

        add_reference_to_stage(usd_path = asset_path, 
                               prim_path = robot_prim
                               )
        
        stage = omni.usd.get_context().get_stage()

        # ── no hand and finger colision ──────────────────────────
        stage = omni.usd.get_context().get_stage()
        robot_root = stage.GetPrimAtPath(robot_prim)

        finger_keywords = [
            "index", "middle", "pinky", "ring", "thumb"
        ]

        for prim in Usd.PrimRange(robot_root):
            path_str = str(prim.GetPath())
            if any(kw in path_str for kw in finger_keywords):
                if prim.HasAPI(UsdPhysics.CollisionAPI):
                    UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Set(False)
                    print(f"[INFO] Collision disabled: {path_str}")

        finger_links = [
            "L_index_proximal", "L_middle_proximal", "L_pinky_proximal",
            "L_ring_proximal", "L_thumb_proximal_base", "L_thumb_proximal",
            "L_thumb_intermediate",
            "R_index_proximal", "R_middle_proximal", "R_pinky_proximal",
            "R_ring_proximal", "R_thumb_proximal_base", "R_thumb_proximal",
            "R_thumb_intermediate",
        ]
        # ─────────────────────────────────────────────────────────────────────

        self._ks_robot = world.scene.add(Robot(prim_path = robot_prim + "/Trunk", name = "T1", position = np.array([-1, 0, 0.675]), orientation = np.array([1, 0, 0, 0])))
        print("Num of degrees of freedom before first reset: " + str(self._ks_robot.num_dof)) # prints None

        self._ks_robot._imu_sensor = IMUSensor(
            prim_path= robot_prim + "/Trunk/Imu_Sensor",
            name="imu",
            dt=physics_dt,
            translation=np.array([0, 0, 0]),
            orientation=np.array([1, 0, 0, 0]),
        )

        self._ks_robot._pub_joint_state = self.create_publisher(sensor_msgs.JointState, '/booster/ros2_k2_joint_states', 10)
        self._ks_robot._msg_joint_state = sensor_msgs.JointState()

        self._ks_robot._pub_real_joint_state = self.create_publisher(sensor_msgs.JointState, '/joint_states', 10)
        self._ks_robot._msg_real_joint_state = sensor_msgs.JointState()

        self._ks_robot._pub_imu = self.create_publisher(sensor_msgs.Imu, '/booster/ros2_k2_imu', 10)
        self._ks_robot._msg_imu = sensor_msgs.Imu()
        
        self._cmd_timestamp = BuiltinTime(sec=0, nanosec=0)


        self.subscription_thread = threading.Thread(target=self.subscription_spin)
        self.subscription_thread.start()

        self._ks_robot._ros_torque_command = np.zeros(len(sim_joint_names)- len(jaw_joint_names))
        self._ks_robot._ros_torque_index = np.zeros(len(sim_joint_names)- len(jaw_joint_names), dtype=int)

        self._ks_robot._sim_joint_name_map = {value : index for index, value in enumerate(sim_joint_names)}
        self._ks_robot._default_joint_name_map = {value : index for index, value in enumerate(default_joint_names)}
        self._frame_count = 0

        self._ks_robot._hand_command_map = {}
        left_hand_command = {}
        for i in range(6):
            left_hand_command[i] = 0.0
        self._ks_robot._hand_command_map[0] = left_hand_command
        right_hand_command = {}
        for i in range(6):
            right_hand_command[i] = 0.0
        self._ks_robot._hand_command_map[1] = right_hand_command

        self._ks_robot._target_hand_command_map = {}
        self._ks_robot._target_hand_command_map[0] = {}
        self._ks_robot._target_hand_command_map[1] = {}


        self._world.reset()

        simulation_app.update()
        self._gait_start = False
        # Creating an ondemand push graph with ROS Clock, everything in the ROS environment must synchronize with this clock
        try:
            keys = og.Controller.Keys
            (self._clock_graph, _, _, _) = og.Controller.edit(
                {
                    "graph_path": "/ROS_Clock",
                    "evaluator_name": "push",
                    "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
                },
                {
                    keys.CREATE_NODES: [
                        ("OnTick", "omni.graph.action.OnTick"),
                        ("readSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                        ("publishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
                    ],
                    keys.CONNECT: [
                        ("OnTick.outputs:tick", "publishClock.inputs:execIn"),
                        ("readSimTime.outputs:simulationTime", "publishClock.inputs:timeStamp"),
                    ],
                },
            )

            (self._ros_camera_graph, _, _, _) = og.Controller.edit(
                {
                    "graph_path": ROS_CAMERA_GRAPH_PATH,
                    "evaluator_name": "push",
                    "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
                },
                {
                    keys.CREATE_NODES: [
                        ("OnTick", "omni.graph.action.OnPlaybackTick"),
                        ("createRenderProduct", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
                        ("cameraHelperRgb", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                        ("createRenderProductDepth", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
                        ("cameraHelperDepth", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                        ("createLeftRenderProduct", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
                        ("cameraLeftHelperRgb", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                        ("createRightRenderProduct", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
                        ("cameraRightHelperRgb", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                        ("createLeftRenderProductInfo", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
                        ("cameraLeftHelperInfo", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                        ("createRightRenderProductInfo", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
                        ("cameraRightHelperInfo", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                        ("createRenderProductDepthCloud", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
                        ("cameraRenderHelperDepthCloud", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                    ],
                    keys.CONNECT: [
                        ("OnTick.outputs:tick", "createRenderProduct.inputs:execIn"),
                        ("createRenderProduct.outputs:execOut", "cameraHelperRgb.inputs:execIn"),
                        ("createRenderProduct.outputs:renderProductPath", "cameraHelperRgb.inputs:renderProductPath"),
                        ("OnTick.outputs:tick", "createRenderProductDepth.inputs:execIn"),
                        ("createRenderProductDepth.outputs:execOut", "cameraHelperDepth.inputs:execIn"),
                        ("createRenderProductDepth.outputs:renderProductPath", "cameraHelperDepth.inputs:renderProductPath"),
                        ("OnTick.outputs:tick", "createLeftRenderProduct.inputs:execIn"),
                        ("createLeftRenderProduct.outputs:execOut", "cameraLeftHelperRgb.inputs:execIn"),
                        ("createLeftRenderProduct.outputs:renderProductPath", "cameraLeftHelperRgb.inputs:renderProductPath"),
                        ("OnTick.outputs:tick", "createRightRenderProduct.inputs:execIn"),
                        ("createRightRenderProduct.outputs:execOut", "cameraRightHelperRgb.inputs:execIn"),
                        ("createRightRenderProduct.outputs:renderProductPath", "cameraRightHelperRgb.inputs:renderProductPath"),
                        ("OnTick.outputs:tick", "createLeftRenderProductInfo.inputs:execIn"),
                        ("createLeftRenderProductInfo.outputs:execOut", "cameraLeftHelperInfo.inputs:execIn"),
                        ("createLeftRenderProductInfo.outputs:renderProductPath", "cameraLeftHelperInfo.inputs:renderProductPath"),
                        ("OnTick.outputs:tick", "createRightRenderProductInfo.inputs:execIn"),
                        ("createRightRenderProductInfo.outputs:execOut", "cameraRightHelperInfo.inputs:execIn"),
                        ("createRightRenderProductInfo.outputs:renderProductPath", "cameraRightHelperInfo.inputs:renderProductPath"),
                        ("OnTick.outputs:tick", "createRenderProductDepthCloud.inputs:execIn"),
                        ("createRenderProductDepthCloud.outputs:execOut", "cameraRenderHelperDepthCloud.inputs:execIn"),
                        ("createRenderProductDepthCloud.outputs:renderProductPath", "cameraRenderHelperDepthCloud.inputs:renderProductPath"),
                    ],
                    keys.SET_VALUES: [
                        ("createRenderProduct.inputs:cameraPrim", robot_prim + "/H2/Realsense/RSD455/Camera_OmniVision_OV9782_Color"),
                        ("cameraHelperRgb.inputs:frameId", "sim_camera"),
                        ("cameraHelperRgb.inputs:topicName", "camera/camera/color/image_raw"),
                        ("cameraHelperRgb.inputs:type", "rgb"),
                        ("createRenderProductDepth.inputs:cameraPrim", robot_prim + "/H2/Realsense/RSD455/Camera_OmniVision_OV9782_Color"),
                        ("cameraHelperDepth.inputs:frameId", "sim_depth_camera"),
                        ("cameraHelperDepth.inputs:topicName", "camera/camera/depth/image_rect_raw"),
                        ("cameraHelperDepth.inputs:type", "depth"),
                        ("createLeftRenderProduct.inputs:cameraPrim", robot_prim + "/H2/Realsense/RSD455/Camera_OmniVision_OV9782_Left"),
                        ("cameraLeftHelperRgb.inputs:frameId", "sim_camera_left"),
                        ("cameraLeftHelperRgb.inputs:topicName", "infra1/image_rect_raw,"),
                        ("cameraLeftHelperRgb.inputs:type", "rgb"),
                        ("createRightRenderProduct.inputs:cameraPrim", robot_prim + "/H2/Realsense/RSD455/Camera_OmniVision_OV9782_Right"),
                        ("cameraRightHelperRgb.inputs:frameId", "sim_camera_right"),
                        ("cameraRightHelperRgb.inputs:topicName", "infra2/image_rect_raw,"),
                        ("cameraRightHelperRgb.inputs:type", "rgb"),
                        ("createLeftRenderProductInfo.inputs:cameraPrim", robot_prim + "/H2/Realsense/RSD455/Camera_OmniVision_OV9782_Left"),
                        ("cameraLeftHelperInfo.inputs:frameId", "sim_camera_left_info"),
                        ("cameraLeftHelperInfo.inputs:topicName", "infra1/camera_info"),
                        ("cameraLeftHelperInfo.inputs:type", "camera_info"),
                        ("createRightRenderProductInfo.inputs:cameraPrim", robot_prim + "/H2/Realsense/RSD455/Camera_OmniVision_OV9782_Right"),
                        ("cameraRightHelperInfo.inputs:frameId", "sim_camera_right_info"),
                        ("cameraRightHelperInfo.inputs:topicName", "infra2/camera_info"),
                        ("cameraRightHelperInfo.inputs:type", "camera_info"),
                        ("createRenderProductDepthCloud.inputs:cameraPrim", robot_prim + "/H2/Realsense/RSD455/Camera_OmniVision_OV9782_Color"),
                        ("cameraRenderHelperDepthCloud.inputs:frameId", "sim_depth_camera"),
                        ("cameraRenderHelperDepthCloud.inputs:topicName", "camera/camera/depth/color/points"),
                        ("cameraRenderHelperDepthCloud.inputs:type", "depth_pcl"),
                    ],
                },
            )
        except Exception as e:
            print("Error creating ROS Clock graph: ", e)
            self._booster_bridge.destroy_node()
            rclpy.shutdown()
            simulation_app.close()
            exit()

    def subscription_spin(self):
        self._ks_robot._sub_joint_state = self.create_subscription(
            sensor_msgs.JointState,
            "/booster/ros2_k2_joint_cmd",
            self.joint_command_callback,
            10)
        self._ks_robot._sub_hand_control = self.create_subscription(
            booster_msgs.HandDdsMsg,
            "/booster_hand",
            self.hand_control_callback,
            10)
        while rclpy.ok():
            rclpy.spin_once(self)

    def setup(self):
        """
        [Summary]

        add physics callback

        """
        self._app_window = omni.appwindow.get_default_app_window()
        self._world.add_physics_callback("robot_sim_step", callback_fn=self.robot_simulation_step)

        float_secs = time.time()
        secs = int(float_secs)
        nsecs = int((float_secs - secs) * 1000000000)
        self._ros_timestamp = rclpy.time.Time(seconds=secs, nanoseconds=nsecs)
        # start ROS publisher and subscribers

        gamepadCameraControlSettingID = "/persistent/app/omniverse/gamepadCameraControl"
        settings = carb.settings.get_settings()
        settings.set_bool(gamepadCameraControlSettingID, False)

    def run(self):
        """
        [Summary]

        Step simulation based on rendering downtime

        """
        print("Booster Run")
        # change to sim running

        t = 0
        push_count = 0

        while simulation_app.is_running():

            start_all = time.perf_counter()

            pysics_time = 0
            render_time = 0

            if not self._gait_start:
                self.publish_ros_data()
                continue

            self._frame_count += 1
            t = t + self._physics_dt

            if self._frame_count % 15 == 0:
                start = time.perf_counter()
                self._world.step(render=True)
                end = time.perf_counter()
                render_time = (end - start) * 1000
                og.Controller.evaluate_sync(self._ros_camera_graph)
            else:
                start = time.perf_counter()
                self._world.step(render=False)
                end = time.perf_counter()
                pysics_time = (end - start) * 1000

            all_time = (time.perf_counter() - start_all) * 1000

            self._gait_start = False

    def publish_ros_data(self):
        """
        [Summary]

        Publish body pose, joint state, imu data

        """

        sec, nanosec = self._ros_timestamp.seconds_nanoseconds()
        builtin_time = BuiltinTime(sec=sec, nanosec=nanosec)
        self._ks_robot._msg_joint_state.header.stamp = builtin_time
        self._ks_robot._msg_real_joint_state.header.stamp = builtin_time
        self._ks_robot._msg_real_joint_state.header.frame_id = "Trunk"

        len = self._ks_robot.dof_names.__len__()
        self._ks_robot._msg_joint_state.position = [0.0] * ( default_joint_names.__len__())
        self._ks_robot._msg_joint_state.velocity = [0.0] * (default_joint_names.__len__())
        self._ks_robot._msg_joint_state.effort = [0.0] * (default_joint_names.__len__())

        self._ks_robot._msg_real_joint_state.position = [0.0] * ( sim_joint_names.__len__())
        self._ks_robot._msg_real_joint_state.velocity = [0.0] * (sim_joint_names.__len__())
        self._ks_robot._msg_real_joint_state.effort = [0.0] * (sim_joint_names.__len__())


        for i in range(len):
            # find the joint name in the default joint name map
            dof_name = self._ks_robot.dof_names[i]

            if dof_name in self._ks_robot._default_joint_name_map:
                id = self._ks_robot._default_joint_name_map[dof_name]
                self._ks_robot._msg_joint_state.position[id] = self._ks_robot.get_joint_positions()[i]
                self._ks_robot._msg_joint_state.velocity[id] = self._ks_robot.get_joint_velocities()[i]
                self._ks_robot._msg_joint_state.effort[id] = self._ks_robot.get_measured_joint_efforts()[i]
            self._ks_robot._msg_real_joint_state.position[i] = self._ks_robot.get_joint_positions()[i]
            # self._ks_robot._msg_real_joint_state.velocity[i] = self._ks_robot.get_joint_velocities()[i]
            # self._ks_robot._msg_real_joint_state.effort[i] = self._ks_robot.get_measured_joint_efforts()[i]
        self._ks_robot._msg_joint_state.name = default_joint_names
        self._ks_robot._msg_real_joint_state.name = sim_joint_names
        
        self._ks_robot._pub_joint_state.publish(self._ks_robot._msg_joint_state)
        self._ks_robot._pub_real_joint_state.publish(self._ks_robot._msg_real_joint_state)

        # set imu data
        self._ks_robot._msg_imu.header.stamp = builtin_time
        lin_acc_0 = self._ks_robot._imu_sensor.get_current_frame()["lin_acc"][0]
        self._ks_robot._msg_imu.linear_acceleration.x = float(self._ks_robot._imu_sensor.get_current_frame()["lin_acc"][0])
        self._ks_robot._msg_imu.linear_acceleration.y = float(self._ks_robot._imu_sensor.get_current_frame()["lin_acc"][1])
        self._ks_robot._msg_imu.linear_acceleration.z = float(self._ks_robot._imu_sensor.get_current_frame()["lin_acc"][2])

        self._ks_robot._msg_imu.angular_velocity.x = float(self._ks_robot._imu_sensor.get_current_frame()["ang_vel"][0])
        self._ks_robot._msg_imu.angular_velocity.y = float(self._ks_robot._imu_sensor.get_current_frame()["ang_vel"][1])
        self._ks_robot._msg_imu.angular_velocity.z = float(self._ks_robot._imu_sensor.get_current_frame()["ang_vel"][2])

        self._ks_robot._msg_imu.orientation.w = float(self._ks_robot._imu_sensor.get_current_frame()["orientation"][0])
        self._ks_robot._msg_imu.orientation.x = float(self._ks_robot._imu_sensor.get_current_frame()["orientation"][1])
        self._ks_robot._msg_imu.orientation.y = float(self._ks_robot._imu_sensor.get_current_frame()["orientation"][2])
        self._ks_robot._msg_imu.orientation.z = float(self._ks_robot._imu_sensor.get_current_frame()["orientation"][3])
        self._ks_robot._pub_imu.publish(self._ks_robot._msg_imu)

        return

    """call backs"""

    def get_finger_joint_index(self, index, seq):
        """
        [Summary]

        Get joint index based on index and seq

        """
        if index == 0:
            if seq == 0:
                # L_pinky_MCP_joint
                return self._ks_robot._sim_joint_name_map['L_pinky_MCP_joint']
            elif seq == 1:
                # L_ring_MCP_joint
                return self._ks_robot._sim_joint_name_map['L_ring_MCP_joint']
            elif seq == 2:
                # L_middle_MCP_joint
                return self._ks_robot._sim_joint_name_map['L_middle_MCP_joint']
            elif seq == 3:
                # L_index_MCP_joint
                return self._ks_robot._sim_joint_name_map['L_index_MCP_joint']
            elif seq == 4:
                # L_thumb_MCP_joint1
                return self._ks_robot._sim_joint_name_map['L_thumb_MCP_joint1']
            elif seq == 5:
                # L_thumb_MCP_joint2
                return self._ks_robot._sim_joint_name_map['L_thumb_MCP_joint2']
        elif index == 1:
            if seq == 0:
                return self._ks_robot._sim_joint_name_map['R_pinky_MCP_joint']
            elif seq == 1:
                return self._ks_robot._sim_joint_name_map['R_ring_MCP_joint']
            elif seq == 2:
                return self._ks_robot._sim_joint_name_map['R_middle_MCP_joint']
            elif seq == 3:
                return self._ks_robot._sim_joint_name_map['R_index_MCP_joint']
            elif seq == 4:
                return self._ks_robot._sim_joint_name_map['R_thumb_MCP_joint1']
            elif seq == 5:
                return self._ks_robot._sim_joint_name_map['R_thumb_MCP_joint2']

    def robot_simulation_step(self, step_size):
        """
        [Summary]

        Call robot update and advance, and tick ros bridge

        """
        
        # self._ks_robot.set_joint_efforts(self._ks_robot._ros_command)
        self._ks_robot.set_joint_efforts(self._ks_robot._ros_torque_command, self._ks_robot._ros_torque_index)

        # start to control hand
        # print("hand command map = ", self._ks_robot._hand_command_map)
        # print("target hand command map = ", self._ks_robot._target_hand_command_map)

        joint_positions = []
        joint_indices = []

        for key in self._ks_robot._hand_command_map:
            hand = self._ks_robot._hand_command_map[key]
            hand_move_finish = True
            for seq in hand:
                pos = hand[seq]

                if key in self._ks_robot._target_hand_command_map and seq in self._ks_robot._target_hand_command_map[key]:
                    pos_delta = self._ks_robot._target_hand_command_map[key][seq] - pos
                    if pos_delta == 0:
                        continue                    
                    if pos_delta > 0 and pos_delta > 0.001:
                        pos += 0.001
                    elif pos_delta < 0 and pos_delta < -0.001:
                        pos -= 0.001
                    elif abs(pos_delta) <= 0.001:
                        pos = self._ks_robot._target_hand_command_map[key][seq]
                    self._ks_robot._hand_command_map[key][seq] = pos
                    target_index = self.get_finger_joint_index(key, seq)
                    # print("set joint position = ", pos, " target index = ", target_index)
                    joint_positions.append(pos)
                    joint_indices.append(target_index)
                    hand_move_finish = False
            if hand_move_finish and key in self._ks_robot._target_hand_command_map:
                # print("hand move finish, clear target hand command map for key = ", key)
                del self._ks_robot._target_hand_command_map[key]
                # print("target hand command map = ", self._ks_robot._target_hand_command_map)
                
        joint_positions_array = np.array(joint_positions)
        joint_indices_array = np.array(joint_indices)
        if joint_positions_array.size > 0:
            self._ks_robot.set_joint_positions(joint_positions_array, joint_indices_array)


        # Tick the ROS Clock
        og.Controller.evaluate_sync(self._clock_graph)


        float_secs = time.time()
        secs = int(float_secs)
        nsecs = int((float_secs - secs) * 1000000000)
        self._ros_timestamp = rclpy.time.Time(seconds=secs, nanoseconds=nsecs)

    def get_finger_angle_in_radians(self, seq, angle):
        """
        根据输入的seq和angle,获取对应的角度信息,并转换为弧度。

        Args:
            seq (int): 手指的序号,范围为0~5。
            angle (int): 手指的角度,范围为0~1000。

        Returns:
            float: 对应的角度信息（弧度）。
        """
        if seq < 0 or seq > 5:
            # raise ValueError("Invalid seq value. It should be between 0 and 5.")
            return -1
        if angle < 0 or angle > 1000:
            # raise ValueError("Invalid angle value. It should be between 0 and 1000.")
            return -1
        angle = 1000 - angle

        if seq == 0:  # Little finger
            degrees =  (angle / 1000) * (90)
        elif seq == 1:  # Ring finger
            degrees =  (angle / 1000) * (90 )
        elif seq == 2:  # Middle finger
            degrees =  (angle / 1000) * (90)
        elif seq == 3:  # Index finger
            degrees =  (angle / 1000) * (90)
        elif seq == 4:  # Thumb bending
            degrees = (angle / 1000) * (20)
        elif seq == 5:  # Thumb rotation
            degrees = (angle / 1000) * (20)
        else:
            raise ValueError("Invalid seq value. It should be between 0 and 5.")

        # 将角度转换为弧度
        radians = math.radians(degrees)
        return radians
    
    def hand_control_callback(self, data: HandDdsMsg):
        """
        This class definition represents a dexterous finger parameter. Different
        dexterous hands have different motion parameters.
        The following parameters all represent a scaling factor. When using them, you
        need to convert them into the corresponding coefficients based on the
        specifications of the dexterous hand you are using.
        For the Inspire RH56 Dexterous Hand:
        - seq: 0~5, represents the sequence of the fingers, as follows:
        -- 0.Little finger, 1.Ring finger, 2.Middle finger, 3.Index finger, 4.Thumb bending, 5.Thumb rotation
        - angle: 0~1000, 0 represents the fully closed state, 1000 represents the
        fully open state. Different fingers have different ranges.
        -- The range 0~1000 of the thumb rotation corresponds to 90-165°
        -- The range 0~1000 of the thumb bending corresponds to -130~53.6°
        -- The range 0~1000 of the other fingers corresponds to 19°-176.7°
        - force: represents the finger's force value, Different fingers have
        different ranges.
        -- The range 0~1500 of the thumb rotation and bending corresponds to 0~1.5kg
        -- he range 0~1000 of the other fingers corresponds to 0~1kg
        - speed: 0~1000, Speed unit: Not specified, The Inspire RH56 dexterous hand
        does not provide a unit, range, or dimension for speed. The speed can only be
        adjusted using values from 0 to 1000, which do not correspond to an absolute
        value        
        """

        print(data)

        # 定义一个字典，键为字符串，值为整数
        hand_command_map = {}

        hand_commands = data.hands_vec
        for hand_command in hand_commands:

            # if hand_command.force_mode:
            #     print("force mode, set postion to 0")
            #     hand_command_map[hand_command.hand_index] = 0.85
            finger_command = {}
            for param in hand_command.hand_param:
                positon = param.angle
                seq = param.seq
                target_rad = self.get_finger_angle_in_radians(seq, positon)
                if target_rad < 0:
                    continue
                finger_command[param.seq] = target_rad
            hand_command_map[hand_command.hand_index] = finger_command

        for key in hand_command_map:
            self._ks_robot._target_hand_command_map[key] = hand_command_map[key]

        print("target hand command map = ", self._ks_robot._target_hand_command_map)

    def joint_command_callback(self, data):
        """
        [Summary]

        Joint command call back, set command torque for the joints

        """

        data_stamp = time_to_float(data.header.stamp)
        last_stamp = time_to_float(self._cmd_timestamp)
        if time_to_float(data.header.stamp) <= time_to_float(self._cmd_timestamp):
            print("skip small time command", str(data.header.stamp), str(self._cmd_timestamp))
            return
        
        for i in range(len(data.effort)):
            # find the joint name in the default joint name map
            name = default_joint_names[i]
            # find the joint id in the sim joint name map
            id = self._ks_robot._sim_joint_name_map[name]
           
            self._ks_robot._ros_torque_command[i] = data.effort[i]
            self._ks_robot._ros_torque_index[i] = id

        self._cmd_timestamp = data.header.stamp
        self._gait_start = True


    """
    Utilities functions.
    """


def main(args=None):
    """
    [Summary]

    The function launches the simulator, creates the robot, and run the simulation steps

    """

    # parse the asset path
    parser = argparse.ArgumentParser(description="Booster Standalone ROS2 RoboCup T1 7DOF Arms")
    parser.add_argument('--asset_path', type=str, help='Path to the asset file')
    parsed_args = parser.parse_args(args)


    # first enable ros node, make sure using simulation time
    rclpy.init()

    physics_downtime = 1 / 500.0
    render_downtime =  1.0 / 200.0

    current_file_dir = os.path.dirname(os.path.abspath(__file__))

    # asset_path = "/home/booster/Documents/RoboCup2024/RoboCup2024/robotcup_bg.usd"
    # asset_path = parsed_args.asset_path if parsed_args.asset_path else current_file_dir + "/scene/RoboCup2024/robot_bg_grab.usd"
    asset_path = parsed_args.asset_path if parsed_args.asset_path else current_file_dir + "/scene/default_bg.usd"
    # asset_path = parsed_args.asset_path if parsed_args.asset_path else current_file_dir + "/scene/default_environment.usd"

    runner = Booster_direct_runner(physics_dt=physics_downtime, render_dt=render_downtime, bg_asset_path=asset_path)
    runner.setup()
    runner._world.reset()
    runner.run()
    runner.subscription_thread.join()
    rclpy.shutdown()
    simulation_app.close()
    return

if __name__ == "__main__":
    main()
