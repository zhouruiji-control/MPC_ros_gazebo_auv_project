#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "gazebo_msgs/srv/get_entity_state.hpp"

#include <chrono>
#include <cstdlib>
#include <ctime>
#include <functional>
#include <memory>
#include <string>

using namespace std::chrono_literals;


// ======================================================
// 生成均匀随机噪声
//
// 输出范围：
// [-noiseAmplitude, noiseAmplitude]
//
// 例如：
// noiseAmplitude = 0.03
//
// 得到：
// [-0.03, 0.03] m
// ======================================================
double noise(double noiseAmplitude)
{
    double randomNumber =
        static_cast<double>(rand()) / RAND_MAX;

    double randomValue =
        randomNumber * 2.0 - 1.0;

    return randomValue * noiseAmplitude;
}


// ======================================================
// 发布Float64
// ======================================================
void publishDouble(
    const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr &publisher,
    double value)
{
    std_msgs::msg::Float64 message;

    message.data = value;

    publisher->publish(message);
}


// ======================================================
// Gazebo深度与速度反馈节点
//
// Gazebo真实状态
//       ↓
// trueDepth / trueVelocity
//       ↓
// 加入测量噪声
//       ↓
// measuredDepth / measuredVelocity
//       ↓
// 一阶低通滤波
//       ↓
// filteredDepth / filteredVelocity
//
// MPC以后使用filtered值
// ======================================================
class ForceDepthFeedbackNode : public rclcpp::Node
{
private:

    // ==================================================
    // Gazebo GetEntityState客户端
    // ==================================================
    rclcpp::Client<
        gazebo_msgs::srv::GetEntityState>::SharedPtr
        getEntityStateClient;


    // ==================================================
    // 定时器
    // ==================================================
    rclcpp::TimerBase::SharedPtr timer;


    // ==================================================
    // 真实状态发布器
    // ==================================================

    // Gazebo真实深度
    rclcpp::Publisher<
        std_msgs::msg::Float64>::SharedPtr
        trueDepthPub;

    // Gazebo真实垂向速度
    rclcpp::Publisher<
        std_msgs::msg::Float64>::SharedPtr
        trueVelocityPub;


    // ==================================================
    // 加噪测量值发布器
    // ==================================================

    rclcpp::Publisher<
        std_msgs::msg::Float64>::SharedPtr
        measuredDepthPub;

    rclcpp::Publisher<
        std_msgs::msg::Float64>::SharedPtr
        measuredVelocityPub;


    // ==================================================
    // 滤波值发布器
    // ==================================================

    rclcpp::Publisher<
        std_msgs::msg::Float64>::SharedPtr
        filteredDepthPub;

    rclcpp::Publisher<
        std_msgs::msg::Float64>::SharedPtr
        filteredVelocityPub;


    // ==================================================
    // Gazebo模型参数
    // ==================================================

    std::string entityName = "simple_auv";
    std::string referenceFrame = "world";


    // ==================================================
    // 噪声参数
    // ==================================================

    // 深度测量噪声
    // 与之前PID保持一致
    double depthNoiseAmplitude = 0.03;

    // 速度测量噪声
    double velocityNoiseAmplitude = 0.01;


    // ==================================================
    // 一阶低通滤波参数
    //
    // filtered =
    // b * previousFiltered
    // + (1-b) * measured
    // ==================================================

    double filterCoefficient = 0.9;


    // ==================================================
    // 滤波状态
    // ==================================================

    double filteredDepth = 0.0;
    double filteredVelocity = 0.0;

    bool filterInitialized = false;


    // 防止上一次Gazebo服务还没有返回
    bool requestInProgress = false;


public:

    ForceDepthFeedbackNode()
        : Node("force_depth_feedback_node")
    {
        // ==================================================
        // 初始化随机数
        // ==================================================
        srand(
            static_cast<unsigned int>(
                time(nullptr)));


        // ==================================================
        // ROS2参数
        // ==================================================

        entityName =
            this->declare_parameter<std::string>(
                "entity_name",
                "simple_auv");

        referenceFrame =
            this->declare_parameter<std::string>(
                "reference_frame",
                "world");

        depthNoiseAmplitude =
            this->declare_parameter<double>(
                "depth_noise_amplitude",
                0.03);

        velocityNoiseAmplitude =
            this->declare_parameter<double>(
                "velocity_noise_amplitude",
                0.01);

        filterCoefficient =
            this->declare_parameter<double>(
                "filter_coefficient",
                0.9);


        // ==================================================
        // 创建真实状态publisher
        // ==================================================

        trueDepthPub =
            this->create_publisher<
                std_msgs::msg::Float64>(
                "/auv/current_trueposition",
                10);

        trueVelocityPub =
            this->create_publisher<
                std_msgs::msg::Float64>(
                "/auv/vertical_velocity",
                10);


        // ==================================================
        // 创建测量值publisher
        // ==================================================

        measuredDepthPub =
            this->create_publisher<
                std_msgs::msg::Float64>(
                "/auv/measured_depth",
                10);

        measuredVelocityPub =
            this->create_publisher<
                std_msgs::msg::Float64>(
                "/auv/measured_velocity",
                10);


        // ==================================================
        // 创建滤波值publisher
        // ==================================================

        filteredDepthPub =
            this->create_publisher<
                std_msgs::msg::Float64>(
                "/auv/filtered_depth",
                10);

        filteredVelocityPub =
            this->create_publisher<
                std_msgs::msg::Float64>(
                "/auv/filtered_velocity",
                10);


        // ==================================================
        // Gazebo服务客户端
        // ==================================================

        getEntityStateClient =
            this->create_client<
                gazebo_msgs::srv::GetEntityState>(
                "/get_entity_state");


        // ==================================================
        // 10 Hz获取状态
        // ==================================================

        timer =
            this->create_wall_timer(
                100ms,
                std::bind(
                    &ForceDepthFeedbackNode::timerCallback,
                    this));


        // ==================================================
        // 启动日志
        // ==================================================

        RCLCPP_INFO(
            this->get_logger(),
            "Gazebo反馈节点启动");

        RCLCPP_INFO(
            this->get_logger(),
            "entity: %s",
            entityName.c_str());

        RCLCPP_INFO(
            this->get_logger(),
            "深度噪声: +/- %.3f m",
            depthNoiseAmplitude);

        RCLCPP_INFO(
            this->get_logger(),
            "速度噪声: +/- %.3f m/s",
            velocityNoiseAmplitude);

        RCLCPP_INFO(
            this->get_logger(),
            "低通滤波系数: %.3f",
            filterCoefficient);
    }


private:

    // ==================================================
    // 定时获取Gazebo状态
    // ==================================================
    void timerCallback()
    {
        // 上一次请求还没有完成
        if (requestInProgress)
        {
            return;
        }


        // Gazebo服务不存在
        if (!getEntityStateClient->service_is_ready())
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "等待 /get_entity_state 服务");

            return;
        }


        // ==================================================
        // 创建请求
        // ==================================================

        auto request =
            std::make_shared<
                gazebo_msgs::srv::GetEntityState::Request>();

        request->name =
            entityName;

        request->reference_frame =
            referenceFrame;

        requestInProgress = true;


        // ==================================================
        // 异步获取Gazebo状态
        // ==================================================

        getEntityStateClient->async_send_request(
            request,

            [this](
                rclcpp::Client<
                    gazebo_msgs::srv::GetEntityState>::SharedFuture
                    future)
            {
                requestInProgress = false;

                auto response =
                    future.get();


                // ==========================================
                // 检查Gazebo返回
                // ==========================================

                if (!response->success)
                {
                    RCLCPP_WARN_THROTTLE(
                        this->get_logger(),
                        *this->get_clock(),
                        2000,
                        "读取simple_auv状态失败");

                    return;
                }


                // ==========================================
                // 1. Gazebo真实状态
                //
                // Gazebo:
                // z向上为正
                //
                // 我们定义：
                // depth向下为正
                //
                // 所以：
                // depth = -z
                // velocity = -vz
                // ==========================================

                double trueDepth =
                    -response->state.pose.position.z;

                double trueVelocity =
                    -response->state.twist.linear.z;


                // ==========================================
                // 2. 加入测量噪声
                // ==========================================

                double measuredDepth =
                    trueDepth
                    + noise(
                        depthNoiseAmplitude);

                double measuredVelocity =
                    trueVelocity
                    + noise(
                        velocityNoiseAmplitude);


                // ==========================================
                // 3. 一阶低通滤波
                // ==========================================

                if (!filterInitialized)
                {
                    filteredDepth =
                        measuredDepth;

                    filteredVelocity =
                        measuredVelocity;

                    filterInitialized =
                        true;
                }
                else
                {
                    filteredDepth =
                        filterCoefficient
                        * filteredDepth
                        +
                        (1.0 - filterCoefficient)
                        * measuredDepth;

                    filteredVelocity =
                        filterCoefficient
                        * filteredVelocity
                        +
                        (1.0 - filterCoefficient)
                        * measuredVelocity;
                }


                // ==========================================
                // 4. 发布真实值
                // ==========================================

                publishDouble(
                    trueDepthPub,
                    trueDepth);

                publishDouble(
                    trueVelocityPub,
                    trueVelocity);


                // ==========================================
                // 5. 发布测量值
                // ==========================================

                publishDouble(
                    measuredDepthPub,
                    measuredDepth);

                publishDouble(
                    measuredVelocityPub,
                    measuredVelocity);


                // ==========================================
                // 6. 发布滤波值
                // ==========================================

                publishDouble(
                    filteredDepthPub,
                    filteredDepth);

                publishDouble(
                    filteredVelocityPub,
                    filteredVelocity);


                // ==========================================
                // 每1秒打印一次
                // ==========================================

                RCLCPP_INFO_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "true=%.3f, measured=%.3f, filtered=%.3f, velocity=%.3f",
                    trueDepth,
                    measuredDepth,
                    filteredDepth,
                    filteredVelocity);
            });
    }
};


// ======================================================
// 主函数
// ======================================================
int main(int argc, char **argv)
{
    rclcpp::init(
        argc,
        argv);

    auto node =
        std::make_shared<
            ForceDepthFeedbackNode>();

    rclcpp::spin(
        node);

    rclcpp::shutdown();

    return 0;
}