import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import Command

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory



def generate_launch_description():

    # 获取功能包安装路径
    auv_description_path = get_package_share_directory(
        "auv_description"
    )

    auv_gazebo_path = get_package_share_directory(
        "auv_gazebo"
    )


    # AUV模型文件
    xacro_file = os.path.join(
        auv_description_path,
        "urdf",
        "simple_auv.urdf.xacro"
    )


    # Gazebo世界文件
    world_file = os.path.join(
        auv_gazebo_path,
        "worlds",
        "empty_underwater.world"
    )



    # =========================================
    # xacro -> robot_description
    # 强制作为字符串传递
    # =========================================

    robot_description = {

        "robot_description":

        ParameterValue(

            Command([
                "xacro",
                " ",
                xacro_file
            ]),

            value_type=str
        )

    }



    # =========================================
    # 启动Gazebo
    # =========================================

    gazebo = ExecuteProcess(

        cmd=[

            "gazebo",

            "--verbose",

            world_file,


            # ROS初始化插件
            "-s",
            "libgazebo_ros_init.so",


            # spawn_entity插件
            "-s",
            "libgazebo_ros_factory.so",


            # 外力插件
            "-s",
            "libgazebo_ros_force_system.so"

        ],

        output="screen"

    )



    # =========================================
    # robot_state_publisher
    # =========================================

    robot_state_publisher = Node(

        package="robot_state_publisher",

        executable="robot_state_publisher",

        name="robot_state_publisher",

        output="screen",

        parameters=[

            robot_description

        ]

    )



    # =========================================
    # 生成AUV模型
    # =========================================

    spawn_auv = Node(

        package="gazebo_ros",

        executable="spawn_entity.py",

        name="spawn_simple_auv",

        output="screen",


        arguments=[

            "-topic",

            "robot_description",


            "-entity",

            "simple_auv",


            "-x",
            "0.0",

            "-y",
            "0.0",

            "-z",
            "0.0",


            "-R",
            "0.0",

            "-P",
            "0.0",

            "-Y",
            "0.0"

        ]

    )



    # 等Gazebo启动后生成模型

    delayed_spawn_auv = TimerAction(

        period=5.0,

        actions=[

            spawn_auv

        ]

    )



    return LaunchDescription([

        gazebo,

        robot_state_publisher,

        delayed_spawn_auv

    ])