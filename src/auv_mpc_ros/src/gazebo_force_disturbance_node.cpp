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
// 【新增】用于MPC外部扰动实验
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
// 【新增】外部扰动力：
// disturbance_force > 0：深度方向向下
//
// Gazebo中向下为z负方向，所以：
// disturbance_force_z = -disturbance_force
//
// 总力：
// total_force_z =
// control_force_z
// + damping_force_z
// + disturbance_force_z
//
// 默认扰动实验：
// 0 ~ 40 s：0 N
// 40 ~ 50 s：+2 N向下
// 50 s以后：0 N
//
// 施力方式：
// 1. 清除上一时刻的持续外力
// 2. 清除完成后施加新的持续外力
// 3. duration.sec = -1，表示持续施力
// ======================================================
class GazeboForceDisturbanceNode : public rclcpp::Node
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


    // ==================================================
    // 【新增】扰动实验参数
    // ==================================================

    // 深度坐标中的扰动力
    //
    // > 0：向下
    // < 0：向上
    double disturbance_force_ = 2.0;

    // 扰动开始时间
    double disturbance_start_time_ = 40.0;

    // 扰动结束时间
    double disturbance_end_time_ = 50.0;

    // MPC实验开始时间
    rclcpp::Time start_time_;

    // 【新增】
    // 是否已经收到第一条MPC控制输出
    // 第一次收到以后才开始扰动实验计时
    bool experiment_started_ = false;


public:
    GazeboForceDisturbanceNode()
        : Node("gazebo_force_disturbance_node")
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
        // 【新增】扰动参数
        // ==================================================
        disturbance_force_ =
            this->declare_parameter<double>(
                "disturbance_force",
                2.0);

        disturbance_start_time_ =
            this->declare_parameter<double>(
                "disturbance_start_time",
                40.0);

        disturbance_end_time_ =
            this->declare_parameter<double>(
                "disturbance_end_time",
                50.0);


        // ==================================================
        // 订阅PID输出
        // ==================================================
        control_output_sub_ =
            this->create_subscription<std_msgs::msg::Float64>(
                "/auv/output",
                10,
                std::bind(
                    &GazeboForceDisturbanceNode::control_output_callback,
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
                    &GazeboForceDisturbanceNode::vertical_velocity_callback,
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
                    &GazeboForceDisturbanceNode::timer_callback,
                    this));

        RCLCPP_INFO(
            this->get_logger(),
            "Gazebo持续外力、阻尼与扰动节点启动成功");

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


        // ==================================================
        // 【新增】打印扰动实验设置
        // ==================================================
        RCLCPP_INFO(
            this->get_logger(),
            "disturbance_force: %.3f N，深度方向向下为正",
            disturbance_force_);

        RCLCPP_INFO(
            this->get_logger(),
            "disturbance time: %.1f s ~ %.1f s",
            disturbance_start_time_,
            disturbance_end_time_);
    }

private:
    // ==================================================
    // 接收PID输出
    // ==================================================
    void control_output_callback(
        const std_msgs::msg::Float64::SharedPtr message)
    {
        // ==================================================
        // 【新增】
        // 第一次收到MPC输出时才开始记录实验时间
        //
        // 原来：
        // 扰动节点一启动就开始计时
        //
        // 现在：
        // 第一次真正收到 /auv/output 后
        // 才令实验时间 t = 0
        //
        // 因此扰动节点40~50 s
        // 基本对应MPC CSV中的40~50 s
        // ==================================================
        if (!experiment_started_)
        {
            start_time_ =
                this->now();

            experiment_started_ = true;

            RCLCPP_INFO(
                this->get_logger(),
                "收到第一条MPC控制输出，扰动实验计时开始");
        }

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
        // 【修改】计算当前MPC实验时间
        //
        // start_time_现在不是节点启动时间，
        // 而是第一次收到MPC控制输出的时间
        // ==================================================
        const double elapsed_time =
            (
                this->now()
                -
                start_time_
            ).seconds();


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
        // 【新增】外部扰动力
        //
        // disturbance_force_的定义：
        //
        // > 0：深度方向向下
        //
        // Gazebo：
        // z负方向向下
        //
        // 所以需要加负号
        //
        // 0 ~ 40 s：
        // disturbance_force_z = 0
        //
        // 40 ~ 50 s：
        // disturbance_force_z = -2 N
        //
        // 50 s以后：
        // disturbance_force_z = 0
        // ==================================================
        double disturbance_force_z =
            0.0;

        if (
            elapsed_time >= disturbance_start_time_
            &&
            elapsed_time < disturbance_end_time_
        )
        {
            disturbance_force_z =
                -disturbance_force_;
        }


        // ==================================================
        // 总外力
        // ==================================================
        double total_force_z =
            control_force_z
            +
            damping_force_z
            +
            disturbance_force_z;

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
             disturbance_force_z,
             total_force_z,
             elapsed_time](
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
                    disturbance_force_z,
                    total_force_z,
                    elapsed_time);
            });
    }

    // ==================================================
    // 第二步：施加新的持续外力
    // ==================================================
    void apply_new_wrench(
        double control_force_z,
        double damping_force_z,
        double disturbance_force_z,
        double total_force_z,
        double elapsed_time)
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
             disturbance_force_z,
             total_force_z,
             elapsed_time](
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

                // ==================================================
                // 【修改】打印扰动力和实验时间
                // ==================================================
                RCLCPP_INFO_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "time=%.1f s, output=%.3f, velocity=%.3f, "
                    "control_force_z=%.3f N, "
                    "damping_force_z=%.3f N, "
                    "disturbance_force_z=%.3f N, "
                    "total_force_z=%.3f N",
                    elapsed_time,
                    control_output_,
                    depth_velocity_,
                    control_force_z,
                    damping_force_z,
                    disturbance_force_z,
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
        std::make_shared<GazeboForceDisturbanceNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}