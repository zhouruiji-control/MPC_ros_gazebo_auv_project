import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ==================================================
    # 基础 Gazebo + simple_auv
    # ==================================================
    simple_auv_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("auv_gazebo"),
                "launch",
                "simple_auv.launch.py",
            )
        )
    )

    # ==================================================
    # 读取 Gazebo 实际深度和垂向速度
    # ==================================================
    force_depth_feedback_node = Node(
        package="auv_lqr_ros",
        executable="force_depth_feedback_node",
        name="force_depth_feedback_node",
        output="screen",
        parameters=[
            {
                "entity_name": "simple_auv",
                "reference_frame": "world",
                "service_name": "/get_entity_state",
            }
        ],
    )

    # ==================================================
    # 将 PID 输出转换为 Gazebo 持续外力
    # ==================================================
    gazebo_force_node = Node(
        package="auv_lqr_ros",
        executable="gazebo_force_node",
        name="gazebo_force_node",
        output="screen",
        parameters=[
            {
                "link_name": "simple_auv::base_link",
                "reference_frame": "",
                "force_scale": 1.0,
                "damping_coefficient": 5.0,
                "maximum_force": 100.0,
            }
        ],
    )

    # ==================================================
    # 只负责 PID 计算
    # ==================================================
    lqr_controller_node = Node(
        package="auv_lqr_ros",
        executable="lqr_controller_node",
        name="lqr_controller_node",
        output="screen",
    )

    return LaunchDescription([

        # 先启动 Gazebo 和模型
        simple_auv_launch,

        # Gazebo 模型生成后读取反馈
        TimerAction(
            period=3.0,
            actions=[
                force_depth_feedback_node,
            ],
        ),

        # 启动持续外力节点
        TimerAction(
            period=4.0,
            actions=[
                gazebo_force_node,
            ],
        ),

        # 最后启动 PID，闭合回路
        TimerAction(
            period=5.0,
            actions=[
                lqr_controller_node,
            ],
        ),
    ])