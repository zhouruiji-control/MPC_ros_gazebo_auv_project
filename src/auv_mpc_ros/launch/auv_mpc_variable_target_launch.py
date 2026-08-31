import os 
 
from launch import LaunchDescription 
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess  # 【新增】
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
                # 【修改】变目标实验初始目标深度10 m
                'target_depth': 10.0, 
 
                # 【修改】单独保存变目标实验数据
                'csv_file': 
                    'auv_mpc_variable_target.csv' 
            } 
        ] 
    ) 
 
 
    # ================================================== 
    # 【新增】30 s后目标深度修改为20 m
    #
    # MPC节点在launch第8秒启动
    #
    # 所以：
    # launch时间38 s
    # =
    # MPC实验时间30 s
    # ================================================== 
 
    target20m = ExecuteProcess( 
        cmd=[ 
            'ros2', 
            'topic', 
            'pub', 
            '--once', 
            '/target_depth', 
            'std_msgs/msg/Float64', 
            '{data: 20.0}' 
        ], 
        output='screen' 
    ) 
 
    delayedTarget20m = TimerAction( 
        period=38.0, 
        actions=[ 
            target20m 
        ] 
    ) 
 
 
    # ================================================== 
    # 【新增】60 s后目标深度修改为15 m
    #
    # MPC节点在launch第8秒启动
    #
    # 所以：
    # launch时间68 s
    # =
    # MPC实验时间60 s
    # ================================================== 
 
    target15m = ExecuteProcess( 
        cmd=[ 
            'ros2', 
            'topic', 
            'pub', 
            '--once', 
            '/target_depth', 
            'std_msgs/msg/Float64', 
            '{data: 15.0}' 
        ], 
        output='screen' 
    ) 
 
    delayedTarget15m = TimerAction( 
        period=68.0, 
        actions=[ 
            target15m 
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
        delayedMpc, 
        delayedTarget20m,       # 【新增】
        delayedTarget15m        # 【新增】
    ])