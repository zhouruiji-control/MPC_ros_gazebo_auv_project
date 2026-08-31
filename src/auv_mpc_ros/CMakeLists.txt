cmake_minimum_required(VERSION 3.8) 
 
project(auv_mpc_ros) 
 
 
find_package(ament_cmake REQUIRED) 
find_package(rclcpp REQUIRED) 
find_package(std_msgs REQUIRED) 
find_package(gazebo_msgs REQUIRED) 
find_package(Eigen3 REQUIRED) 
find_package(OsqpEigen REQUIRED) 
 
 
# ========================= 
# MPC控制节点 
# 替代原PID控制节点 
# ========================= 
 
add_executable( 
  mpc_controller_node 
  src/mpc_controller_node.cpp 
) 
target_link_libraries( 
  mpc_controller_node 
  Eigen3::Eigen 
) 
 
 
ament_target_dependencies( 
  mpc_controller_node 
  rclcpp 
  std_msgs 
) 
target_link_libraries( 
  mpc_controller_node 
  OsqpEigen::OsqpEigen 
) 
 
 
 
# ========================= 
# 受力版：读取Gazebo真实深度 
# 保留 
# ========================= 
 
add_executable( 
  force_depth_feedback_node 
  src/force_depth_feedback_node.cpp 
) 
 
 
ament_target_dependencies( 
  force_depth_feedback_node 
  rclcpp 
  std_msgs 
  gazebo_msgs 
) 
 
 
 
# ========================= 
# 受力版：控制输出转换为Gazebo外力 
# 保留 
# ========================= 
 
add_executable( 
  gazebo_force_node 
  src/gazebo_force_node.cpp 
) 
 
 
ament_target_dependencies( 
  gazebo_force_node 
  rclcpp 
  std_msgs 
  gazebo_msgs 
) 
 
 
 
# ========================= 
# 【新增】扰动实验：控制输出、阻尼和外部扰动力 
# ========================= 
 
add_executable( 
  gazebo_force_disturbance_node 
  src/gazebo_force_disturbance_node.cpp 
) 
 
 
ament_target_dependencies( 
  gazebo_force_disturbance_node 
  rclcpp 
  std_msgs 
  gazebo_msgs 
) 
 
 
 
# ========================= 
# 安装节点 
# ========================= 
 
install(TARGETS 
 
  mpc_controller_node 
 
  force_depth_feedback_node 
 
  gazebo_force_node 
 
  # 【新增】
  gazebo_force_disturbance_node 
 
  DESTINATION lib/${PROJECT_NAME} 
 
) 
 
 
 
# ========================= 
# 安装launch 
# ========================= 
 
install( 
  DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME} 
) 
 
 
ament_package()