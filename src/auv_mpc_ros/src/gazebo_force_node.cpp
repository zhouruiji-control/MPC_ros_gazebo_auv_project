#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include "gazebo_msgs/srv/apply_link_wrench.hpp"
#include "gazebo_msgs/srv/link_request.hpp"

#include <chrono>
#include <exception>
#include <functional>
#include <memory>
#include <string>

using namespace std::chrono_literals;


// ======================================================
// 数值限幅
// ======================================================
double limit_value(
    double value,
    double maximum,
    double minimum)
{
    if (value > maximum)
    {
        return maximum;
    }

    if (value < minimum)
    {
        return minimum;
    }

    return value;
}


// ======================================================
// Gazebo外力与垂向阻尼节点
//
// 订阅：
// /auv/output
// /auv/vertical_velocity
//
// 坐标规定：
// control_output > 0：要求AUV下潜
// depth_velocity > 0：AUV正在下潜
//
// Gazebo世界坐标：
// z正方向向上
//
// PID控制力：
// control_force_z = -force_scale * control_output
//
// 阻尼力：
// damping_force_z = damping_coefficient * depth_velocity
//
// 总力：
// total_force_z = control_force_z + damping_force_z
//
// 施力方式：
// 1. 清除上一时刻的持续外力
// 2. 清除完成后施加新的持续外力
// 3. duration.sec = -1，表示持续施力
// ======================================================
class GazeboForceNode : public rclcpp::Node
{
private:
    // PID输出订阅器
    rclcpp::Subscription<
        std_msgs::msg::Float64>::SharedPtr
        control_output_sub_;

    // 垂向速度订阅器
    rclcpp::Subscription<
        std_msgs::msg::Float64>::SharedPtr
        vertical_velocity_sub_;

    // 清除旧外力服务客户端
    rclcpp::Client<
        gazebo_msgs::srv::LinkRequest>::SharedPtr
        clear_wrench_client_;

    // 施加新外力服务客户端
    rclcpp::Client<
        gazebo_msgs::srv::ApplyLinkWrench>::SharedPtr
        apply_wrench_client_;

    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;

    // PID控制输出
    double control_output_ = 0.0;

    // 深度方向速度，向下为正
    double depth_velocity_ = 0.0;

    // 是否收到PID输出
    bool received_control_output_ = false;

    // 是否收到速度
    bool received_vertical_velocity_ = false;

    // 防止一轮清力和施力尚未完成时重复发送
    bool update_in_progress_ = false;

    // 控制输出转换为实际力的比例
    double force_scale_ = 1.0;

    // 垂向线性阻尼系数，单位N·s/m
    double damping_coefficient_ = 5.0;

    // 总外力限幅，单位N
    double maximum_force_ = 100.0;

    // Gazebo完整link名称
    std::string link_name_;

    // 外力参考坐标系
    std::string reference_frame_;

public:
    GazeboForceNode()
        : Node("gazebo_force_node")
    {
        // ==================================================
        // ROS2参数
        // ==================================================
        force_scale_ =
            this->declare_parameter<double>(
                "force_scale",
                1.0);

        damping_coefficient_ =
            this->declare_parameter<double>(
                "damping_coefficient",
                5.0);

        maximum_force_ =
            this->declare_parameter<double>(
                "maximum_force",
                100.0);

        link_name_ =
            this->declare_parameter<std::string>(
                "link_name",
                "simple_auv::base_link");

        // 空字符串表示世界惯性坐标系
        reference_frame_ =
            this->declare_parameter<std::string>(
                "reference_frame",
                "");

        // ==================================================
        // 订阅PID输出
        // ==================================================
        control_output_sub_ =
            this->create_subscription<std_msgs::msg::Float64>(
                "/auv/output",
                10,
                std::bind(
                    &GazeboForceNode::control_output_callback,
                    this,
                    std::placeholders::_1));

        // ==================================================
        // 订阅垂向速度
        // ==================================================
        vertical_velocity_sub_ =
            this->create_subscription<std_msgs::msg::Float64>(
                "/auv/vertical_velocity",
                10,
                std::bind(
                    &GazeboForceNode::vertical_velocity_callback,
                    this,
                    std::placeholders::_1));

        // ==================================================
        // 创建Gazebo服务客户端
        // ==================================================
        clear_wrench_client_ =
            this->create_client<
                gazebo_msgs::srv::LinkRequest>(
                "/clear_link_wrenches");

        apply_wrench_client_ =
            this->create_client<
                gazebo_msgs::srv::ApplyLinkWrench>(
                "/apply_link_wrench");

        // ==================================================
        // 10 Hz更新外力
        // ==================================================
        timer_ =
            this->create_wall_timer(
                100ms,
                std::bind(
                    &GazeboForceNode::timer_callback,
                    this));

        RCLCPP_INFO(
            this->get_logger(),
            "Gazebo持续外力与阻尼节点启动成功");

        RCLCPP_INFO(
            this->get_logger(),
            "link_name: %s",
            link_name_.c_str());

        RCLCPP_INFO(
            this->get_logger(),
            "force_scale: %.3f",
            force_scale_);

        RCLCPP_INFO(
            this->get_logger(),
            "damping_coefficient: %.3f N*s/m",
            damping_coefficient_);

        RCLCPP_INFO(
            this->get_logger(),
            "maximum_force: %.3f N",
            maximum_force_);
    }

private:
    // ==================================================
    // 接收PID输出
    // ==================================================
    void control_output_callback(
        const std_msgs::msg::Float64::SharedPtr message)
    {
        control_output_ = message->data;
        received_control_output_ = true;
    }

    // ==================================================
    // 接收深度方向速度
    // ==================================================
    void vertical_velocity_callback(
        const std_msgs::msg::Float64::SharedPtr message)
    {
        depth_velocity_ = message->data;
        received_vertical_velocity_ = true;
    }

    // ==================================================
    // 定时更新外力
    // ==================================================
    void timer_callback()
    {
        // 必须先收到PID输出
        if (!received_control_output_)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "等待PID输出 /auv/output");

            return;
        }

        // 尚未收到速度时，阻尼暂时按0计算
        // 但仍允许PID控制力生效
        if (!received_vertical_velocity_)
        {
            depth_velocity_ = 0.0;

            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "尚未收到 /auv/vertical_velocity，阻尼暂时为0");
        }

        // 上一轮清力和施力尚未完成
        if (update_in_progress_)
        {
            return;
        }

        // 检查两个Gazebo服务
        if (!clear_wrench_client_->service_is_ready())
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "等待Gazebo服务 /clear_link_wrenches");

            return;
        }

        if (!apply_wrench_client_->service_is_ready())
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "等待Gazebo服务 /apply_link_wrench");

            return;
        }

        // ==================================================
        // PID控制力
        //
        // output > 0表示要求下潜
        // Gazebo z负方向表示向下
        // ==================================================
        const double control_force_z =
            -force_scale_ * control_output_;

        // ==================================================
        // 垂向阻尼力
        //
        // depth_velocity > 0：AUV正在下潜
        // 阻尼力应向上，所以Gazebo z方向为正
        //
        // depth_velocity < 0：AUV正在上浮
        // 阻尼力应向下，所以Gazebo z方向为负
        // ==================================================
        const double damping_force_z =
            damping_coefficient_ * depth_velocity_;

        // ==================================================
        // 总外力
        // ==================================================
        double total_force_z =
            control_force_z + damping_force_z;

        total_force_z =
            limit_value(
                total_force_z,
                maximum_force_,
                -maximum_force_);

        // 开始本轮更新
        update_in_progress_ = true;

        // ==================================================
        // 第一步：清除上一时刻的持续力
        // ==================================================
        auto clear_request =
            std::make_shared<
                gazebo_msgs::srv::LinkRequest::Request>();

        clear_request->link_name =
            link_name_;

        clear_wrench_client_->async_send_request(
            clear_request,
            [this,
             control_force_z,
             damping_force_z,
             total_force_z](
                rclcpp::Client<
                    gazebo_msgs::srv::LinkRequest>::
                    SharedFuture clear_future)
            {
                try
                {
                    // LinkRequest响应没有字段，但需要取回响应，
                    // 用于捕获服务调用异常
                    clear_future.get();
                }
                catch (const std::exception &error)
                {
                    update_in_progress_ = false;

                    RCLCPP_ERROR(
                        this->get_logger(),
                        "清除旧外力异常: %s",
                        error.what());

                    return;
                }

                // 清除完成后再施加新力
                apply_new_wrench(
                    control_force_z,
                    damping_force_z,
                    total_force_z);
            });
    }

    // ==================================================
    // 第二步：施加新的持续外力
    // ==================================================
    void apply_new_wrench(
        double control_force_z,
        double damping_force_z,
        double total_force_z)
    {
        auto apply_request =
            std::make_shared<
                gazebo_msgs::srv::ApplyLinkWrench::Request>();

        apply_request->link_name =
            link_name_;

        apply_request->reference_frame =
            reference_frame_;

        // 力作用在link原点
        apply_request->reference_point.x = 0.0;
        apply_request->reference_point.y = 0.0;
        apply_request->reference_point.z = 0.0;

        // 只施加z方向力
        apply_request->wrench.force.x = 0.0;
        apply_request->wrench.force.y = 0.0;
        apply_request->wrench.force.z = total_force_z;

        // 不施加力矩
        apply_request->wrench.torque.x = 0.0;
        apply_request->wrench.torque.y = 0.0;
        apply_request->wrench.torque.z = 0.0;

        // 立即开始施力
        apply_request->start_time.sec = 0;
        apply_request->start_time.nanosec = 0;

        // -1表示持续施力，直到下一次清除
        apply_request->duration.sec = -1;
        apply_request->duration.nanosec = 0;

        apply_wrench_client_->async_send_request(
            apply_request,
            [this,
             control_force_z,
             damping_force_z,
             total_force_z](
                rclcpp::Client<
                    gazebo_msgs::srv::ApplyLinkWrench>::
                    SharedFuture apply_future)
            {
                update_in_progress_ = false;

                try
                {
                    const auto response =
                        apply_future.get();

                    if (!response->success)
                    {
                        RCLCPP_WARN_THROTTLE(
                            this->get_logger(),
                            *this->get_clock(),
                            2000,
                            "施加持续外力失败: %s",
                            response->status_message.c_str());

                        return;
                    }
                }
                catch (const std::exception &error)
                {
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "施加持续外力异常: %s",
                        error.what());

                    return;
                }

                RCLCPP_INFO_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "output=%.3f, velocity=%.3f, "
                    "control_force_z=%.3f N, "
                    "damping_force_z=%.3f N, "
                    "total_force_z=%.3f N",
                    control_output_,
                    depth_velocity_,
                    control_force_z,
                    damping_force_z,
                    total_force_z);
            });
    }
};


// ======================================================
// 主函数
// ======================================================
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<GazeboForceNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}