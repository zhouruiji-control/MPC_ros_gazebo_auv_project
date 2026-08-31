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
    # 【修改】Gazebo力节点
    #
    # 普通实验：
    # gazebo_force_node
    #
    # 本实验：
    # gazebo_force_disturbance_node
    # ================================================== 
 
    forceNode = Node( 
        package='auv_mpc_ros', 
        executable='gazebo_force_disturbance_node', 
        name='gazebo_force_disturbance_node', 
        output='screen', 
 
        parameters=[ 
            { 
                'force_scale': 1.0, 
 
                'damping_coefficient': 5.0, 
 
                'maximum_force': 100.0, 
 
                # 【新增】
                # 深度方向扰动力
                # 正值表示向下
                'disturbance_force': 2.0, 
 
                # 【新增】
                # MPC实验40 s开始扰动
                'disturbance_start_time': 40.0, 
 
                # 【新增】
                # MPC实验50 s结束扰动
                'disturbance_end_time': 50.0 
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
 
                # 【修改】
                # 单独保存扰动实验数据
                'csv_file': 
                    'auv_mpc_disturbance_20m.csv' 
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
 
    # ==================================================
    # 【修改】
    #
    # 原普通ForceNode在7 s启动。
    #
    # 这里让扰动ForceNode与MPC同时在8 s启动，
    # 这样扰动节点自己的40~50 s，
    # 基本对应MPC CSV中的40~50 s。
    # ==================================================
 
    delayedForce = TimerAction( 
        period=8.0, 
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