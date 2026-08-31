import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ==================================================
    # Gazebo launch
    # ==================================================

    auvGazeboPath = get_package_share_directory(
        'auv_gazebo'
    )

    gazeboLaunchFile = os.path.join(
        auvGazeboPath,
        'launch',
        'simple_auv.launch.py'
    )

    gazeboLaunch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            gazeboLaunchFile
        )
    )


    # ==================================================
    # 反馈 + 噪声 + 滤波
    # ==================================================

    feedbackNode = Node(
        package='auv_mpc_ros',
        executable='force_depth_feedback_node',
        name='force_depth_feedback_node',
        output='screen',

        parameters=[
            {
                'entity_name': 'simple_auv',

                'reference_frame': 'world',

                'depth_noise_amplitude': 0.03,

                'velocity_noise_amplitude': 0.01,

                'filter_coefficient': 0.9
            }
        ]
    )


    # ==================================================
    # Gazebo力节点
    # ==================================================

    forceNode = Node(
        package='auv_mpc_ros',
        executable='gazebo_force_node',
        name='gazebo_force_node',
        output='screen',

        parameters=[
            {
                'force_scale': 1.0,

                'damping_coefficient': 5.0,

                'maximum_force': 100.0
            }
        ]
    )


    # ==================================================
    # MPC
    # ==================================================

    mpcNode = Node(
        package='auv_mpc_ros',
        executable='mpc_controller_node',
        name='mpc_controller_node',
        output='screen',

        parameters=[
            {
                'target_depth': 20.0,

                'csv_file':
                    'auv_mpc_20m.csv'
            }
        ]
    )


    # ==================================================
    # 延时启动
    # ==================================================

    delayedFeedback = TimerAction(
        period=6.0,
        actions=[
            feedbackNode
        ]
    )

    delayedForce = TimerAction(
        period=7.0,
        actions=[
            forceNode
        ]
    )

    delayedMpc = TimerAction(
        period=8.0,
        actions=[
            mpcNode
        ]
    )


    return LaunchDescription([
        gazeboLaunch,
        delayedFeedback,
        delayedForce,
        delayedMpc
    ])