#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <iostream>
#include <fstream>
#include <cmath>
#include <string>
#include <eigen3/Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>
#include <eigen3/Eigen/Sparse>

// ======================================================
// 【新加】随机噪声需要
// ======================================================
#include <cstdlib>
#include <ctime>

using namespace std;
using namespace std::chrono_literals;


// ======================================================
// 限幅函数
// ======================================================
double limit(double value, double max_value, double min_value)
{
    if (value > max_value)
    {
        return max_value;
    }

    if (value < min_value)
    {
        return min_value;
    }

    return value;
}


// ======================================================
// 【新加】均匀随机测量噪声
//
// noise_amplitude = 0.03 时：
// 噪声范围为 [-0.03, 0.03]
// ======================================================
double noise(double noise_amplitude)
{
    double randomnum=(double)rand()/RAND_MAX;
    double randomnum_f1_1=randomnum*2-1;
    double noise_value=randomnum_f1_1*noise_amplitude;
    return noise_value;
}


// ======================================================
// MPC控制器类
// ======================================================

class MPC
{
private:
    double mass=20.0;
    double damping=5;
    double mpcDt=0.1;
    Eigen::Matrix2d A;//连续矩阵
    Eigen::Vector2d B;
    Eigen::Matrix2d Ad;//离散时间矩阵
    Eigen::Vector2d Bd;
    Eigen::Matrix2d Q;
    double R=1;
    int N=30;
    Eigen::MatrixXd F;
    Eigen::MatrixXd G;
    Eigen::MatrixXd Qbar;
    Eigen::MatrixXd Rbar;
    Eigen::MatrixXd P_qp;
    Eigen::VectorXd q_qp;
Eigen::MatrixXd A_qp;
Eigen::VectorXd lower_qp;
Eigen::VectorXd upper_qp;
// 【新增】OSQP使用稀疏矩阵
Eigen::SparseMatrix<double> P_sparse;
Eigen::SparseMatrix<double> A_qp_sparse;
// 【新增】OSQP求解器
OsqpEigen::Solver solver;
bool solver_ready=false;

    double previous_output = 0.0;
    // 输出变化率限制
    double output_rate = 3.0;
    // 输出限幅
    double output_max = 5.0;
    double output_min = -5.0;
public:MPC(){
            A << 0.0, 1.0,0.0, -damping / mass;
            B<<0.0,1/mass;
            Q << 10.0, 0.0,0.0, 2.0;
            R = 1.0;
        //离散化先求Ad,Bd
        Ad= Eigen::Matrix2d::Identity()+A*mpcDt;
        Bd=B*mpcDt;
        
        //构造MPC预测矩阵F、G
        buildPredictionMatrices();

        // 构造MPC扩展权重矩阵Qbar、Rbar
        buildWeightMatrices();

        //计算QP目标函数中的P
        calculateP();
// 【新增】构造约束矩阵a
buildConstraintMatrix();

// 【新增】初始化第一次约束上下界L,U
updateConstraintBounds(mpcDt);

// 【新增】初始化OSQP求解器
initializeQPSolver();

        // 打印结果
        cout << endl;
        cout << "==============================" << endl;
        cout << "MPC calculation finished" << endl;

        cout << "mass = " << mass << endl;
        cout << "damping = " << damping << endl;
        cout << "dt = " << mpcDt << endl;

        cout << "A =" << endl;
        cout << A << endl;

        cout << "B =" << endl;
        cout << B << endl;
        
cout << "Ad =" << endl;
cout << Ad << endl;

cout << "Bd =" << endl;
cout << Bd << endl;

        cout << "Q =" << endl;
        cout << Q << endl;

        cout << "R = " << R << endl;

     

        cout << "==============================" << endl;
        cout << endl;
    }
public:
void buildPredictionMatrices(){
F=Eigen::MatrixXd::Zero(2*N,2);
Eigen::Matrix2d A_power=Eigen::Matrix2d::Identity();
for(int i=0;i<30;i++){
    A_power=A_power*Ad;
     F.block(2*i,0,2,2)=A_power;
};
G=Eigen::MatrixXd::Zero(2*N,N);
for(int i=0;i<N;i++){
    for(int j=0;j<=i;j++){
        Eigen::Matrix2d A_power_G=
        Eigen::Matrix2d::Identity();
        for(int k=0;k<i-j;k++){
            A_power_G=A_power_G*Ad;};
        G.block(2*i,j,2,1)=A_power_G*Bd;
    };
};
};
//Q,R
public:
void buildWeightMatrices(){
Qbar=
Eigen::MatrixXd::Zero(2*N,2*N);
for(int i=0;i<N;i++){
    Qbar.block(2*i,2*i,2,2)=Q;};
Rbar=Eigen::MatrixXd::Identity(N,N)*R;};
//P
public:void calculateP(){
P_qp=2.0*(G.transpose()*Qbar*G+Rbar);}
// 【新增】构造QP约束矩阵
public:void buildConstraintMatrix(){
A_qp=Eigen::MatrixXd::Zero(2*N,N);
//前N行：推力大小约束
A_qp.block(0,0,N,N)=
Eigen::MatrixXd::Identity(N,N);
//后N行：推力变化率约束
for(int i=0;i<N;i++){
    if(i==0){
        A_qp(N+i,i)=1.0;}
    else{
        A_qp(N+i,i-1)=-1.0;
        A_qp(N+i,i)=1.0;
    };
};
};

public:void updateConstraintBounds(double dt){
lower_qp=Eigen::VectorXd::Zero(2*N);
upper_qp=Eigen::VectorXd::Zero(2*N);
//前N个：推力限制
for(int i=0;i<N;i++){
    lower_qp(i)=output_min;
    upper_qp(i)=output_max;
};
double maxChange=output_rate*dt;
//第一个控制量相对于上一周期真实输出
lower_qp(N)=previous_output-maxChange;

upper_qp(N)=previous_output+maxChange;

//后面预测控制量之间的变化率
for(int i=1;i<N;i++){

    lower_qp(N+i)=-maxChange;

    upper_qp(N+i)=maxChange;

};

};
// 【新增】初始化OSQP
public:void initializeQPSolver(){

//初始化q
q_qp=Eigen::VectorXd::Zero(N);

//Dense矩阵转换为Sparse矩阵
P_sparse=P_qp.sparseView();

A_qp_sparse=A_qp.sparseView();

P_sparse.makeCompressed();

A_qp_sparse.makeCompressed();


//开启warm start
solver.settings()->setWarmStart(true);
// 【新增】提高OSQP求解精度
solver.settings()->setAbsoluteTolerance(1.0e-5);
solver.settings()->setRelativeTolerance(1.0e-5);

//告诉求解器变量数量
solver.data()->setNumberOfVariables(N);


//告诉求解器约束数量
solver.data()->setNumberOfConstraints(2*N);


bool success=true;

success=
success &&
solver.data()->setHessianMatrix(P_sparse);

success=
success &&
solver.data()->setGradient(q_qp);

success=
success &&
solver.data()->setLinearConstraintsMatrix(A_qp_sparse);

success=
success &&
solver.data()->setLowerBound(lower_qp);

success=
success &&
solver.data()->setUpperBound(upper_qp);


if(!success){

    cerr<<"OSQP data setting failed"<<endl;

    solver_ready=false;

    return;
};


if(!solver.initSolver()){

    cerr<<"OSQP initialization failed"<<endl;

    solver_ready=false;

    return;
};


solver_ready=true;

cout<<"OSQP initialization successful"<<endl;

};
public:
double output(double target_depth,double current_depth,double velocity,double dt){
    Eigen::Vector2d e;
    e<<current_depth - target_depth,velocity;
   q_qp=2.0*G.transpose()*Qbar*F*e;
// 【新增】根据上一周期真实输出更新约束
updateConstraintBounds(dt);


// 【新增】检查OSQP是否初始化成功
if(!solver_ready){

    cerr<<"OSQP solver is not ready"<<endl;

    return previous_output;

};


// 【新增】更新q
if(!solver.updateGradient(q_qp)){

    cerr<<"OSQP gradient update failed"<<endl;

    return previous_output;

};


// 【新增】更新约束上下界
if(!solver.updateBounds(lower_qp,upper_qp)){

    cerr<<"OSQP bounds update failed"<<endl;

    return previous_output;

};


// 【新增】QP求解
if(
    solver.solveProblem()
    !=
    OsqpEigen::ErrorExitFlag::NoError
){

    cerr<<"OSQP solve failed"<<endl;

    return previous_output;

};


// 【新增】得到未来N步最优控制序列
Eigen::VectorXd Uoptimal=
solver.getSolution().cast<double>();


// MPC只执行第一项
double control =Uoptimal(0);
    
previous_output =control;

return control;
};};



// ======================================================
// 只负责控制的MPC节点
//
// 默认目标深度：10 m

//
// 订阅：
// /target_depth
// /auv/current_trueposition
//
// 发布：
// /auv/output
// /auv/error

//
// 不计算AUV位置
// 不包含auvmodel.update()
// ======================================================
class MPCControllerOnlyNode : public rclcpp::Node
{
private:
    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;

    // 目标深度订阅器
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr
        target_depth_sub_;

    // Gazebo当前深度订阅器
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr
        current_depth_sub_;
    // Gazebo速度订阅器
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_sub_;
    // 控制输出发布器
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        output_pub_;

    // 误差发布器
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        error_pub_;

;

    // 默认目标深度10 m
    double target_depth = 20.0;

    // Gazebo反馈的当前深度
    double current_depth = 0.0;
    double velocity=0.0;


    // ======================================================
    // 【新加】测量噪声与低通滤波
    // ======================================================

    // 加噪后的深度
    double measured_depth = 0.0;

    // 滤波后的深度
    double filtered_depth = 0.0;

    // 加噪后的速度
    double measured_velocity = 0.0;

    // 滤波后的速度
    double filtered_velocity = 0.0;

    // 深度噪声幅值
    // 与之前PID一致：±0.03 m
    double depth_noise_amplitude = 0.03;

    // 速度噪声幅值
    double velocity_noise_amplitude = 0.01;

    // 一阶低通滤波系数
    double filter_coefficient = 0.8;

    // 判断滤波器是否第一次运行
    bool filter_initialized = false;


    // 控制周期0.05
    double dt = 0.1;

    // 是否收到Gazebo深度
    bool received_current_depth = false;
    bool received_velocity=false;
    MPC controller;


    // CSV记录时间步
    std::size_t step = 0;

    // CSV文件
    ofstream file;

    // 默认CSV文件名
    string csv_file_path = "auv_mpc_force.csv";

public:
    MPCControllerOnlyNode()
        : Node("MPC_controller_only_node")
    {


        // ======================================================
        // 【新加】初始化随机数
        // ======================================================
        srand(time(0));


        // ==================================================
        // 声明并读取ROS2参数
        // ==================================================
        target_depth =
            this->declare_parameter<double>(
                "target_depth",
                20.0);
                

        csv_file_path =
            this->declare_parameter<string>(
                "csv_file",
                "auv_mpc_force.csv");



        // ==================================================
        // 订阅目标深度
        //
        // 没有收到话题时使用默认值20 m
        // 收到消息后使用新的目标深度
        // ==================================================
        target_depth_sub_ =
            this->create_subscription<std_msgs::msg::Float64>(
                "/target_depth",
                10,
                std::bind(
                    &MPCControllerOnlyNode::target_depth_callback,
                    this,
                    std::placeholders::_1));
        velocity_sub_ =
            this->create_subscription<std_msgs::msg::Float64>(
             "/auv/vertical_velocity",
            10,
                std::bind(
                &MPCControllerOnlyNode::velocity_callback,
            this,
            std::placeholders::_1));

        // ==================================================
        // 订阅Gazebo当前深度
        // ==================================================
        current_depth_sub_ =
            this->create_subscription<std_msgs::msg::Float64>(
                "/auv/current_trueposition",
                10,
                std::bind(
                    &MPCControllerOnlyNode::current_depth_callback,
                    this,
                    std::placeholders::_1));

        // ==================================================
        // 发布控制输出
        // ==================================================
        output_pub_ =
            this->create_publisher<std_msgs::msg::Float64>(
                "/auv/output",
                10);

        // 发布误差
        error_pub_ =
            this->create_publisher<std_msgs::msg::Float64>(
                "/auv/error",
                10);



        // ==================================================
        // 创建CSV文件
        // std::ios::out表示每次启动覆盖旧文件
        // ==================================================
        file.open(
            csv_file_path,
            std::ios::out);

        if (!file.is_open())
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "CSV文件打开失败: %s",
                csv_file_path.c_str());
        }
        else
        {
   file
<< "time,"
<< "target_depth,"
<< "current_depth,"
<< "velocity,"
<< "error,"
<< "output"

// ======================================================
// 【新加】CSV继续追加测量值和滤波值
// 原来的6列没有删除
// ======================================================
<< ",measured_depth"
<< ",filtered_depth"
<< ",measured_velocity"
<< ",filtered_velocity"

<< endl;

            RCLCPP_INFO(
                this->get_logger(),
                "CSV文件已创建: %s",
                csv_file_path.c_str());
        }

        // ==================================================
        // 创建5 Hz控制定时器
        // ==================================================
        timer_ =
            this->create_wall_timer(
                100ms,
                std::bind(
                    &MPCControllerOnlyNode::timer_callback,
                    this));

        RCLCPP_INFO(
            this->get_logger(),
            "MPC控制器节点启动成功");

        RCLCPP_INFO(
            this->get_logger(),
            "目标深度: %.3f m",
            target_depth);

        RCLCPP_INFO(
            this->get_logger(),
            "订阅目标深度: /target_depth");

        RCLCPP_INFO(
            this->get_logger(),
            "订阅Gazebo深度: /auv/current_trueposition");

        RCLCPP_INFO(
            this->get_logger(),
            "发布控制输出: /auv/output");
        RCLCPP_INFO(this->get_logger(),
            "订阅Gazebo速度: /auv/vertical_velocity");
    }

~MPCControllerOnlyNode() noexcept override
{
    if (file.is_open())
    {
        file.flush();
        file.close();
    }
}

private:
    // ==================================================
    // 接收目标深度
    // ==================================================
    void target_depth_callback(
        const std_msgs::msg::Float64::SharedPtr msg)
    {
        target_depth = msg->data;

    }
void velocity_callback(
const std_msgs::msg::Float64::SharedPtr msg)
{
    velocity = msg->data;
    received_velocity = true;
}
    // ==================================================
    // 接收Gazebo当前深度
    // ==================================================
    void current_depth_callback(
        const std_msgs::msg::Float64::SharedPtr msg)
    {
        current_depth = msg->data;
        received_current_depth = true;
    }

    // ==================================================
    // 发布Float64数据
    // ==================================================
    void publish_double(
        const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr &pub,
        double value)
    {
        std_msgs::msg::Float64 msg;
        msg.data = value;

        pub->publish(msg);
    }

    // ==================================================
    // MPC控制循环
    // ==================================================
    void timer_callback()
    {
      if (!received_current_depth || !received_velocity)
{
    RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "等待深度或速度反馈");

    return;
};

        double time =
            static_cast<double>(step) * dt;


        // ======================================================
        // 【新加1】保存Gazebo真实深度和真实速度
        //
        // 后面MPC临时使用滤波值
        // 控制计算结束后再恢复
        // ======================================================
        double true_depth = current_depth;
        double true_velocity = velocity;


        // ======================================================
        // 【新加2】加入测量噪声
        //
        // measured_depth：
        // 真实深度 + [-0.03,0.03] m随机噪声
        //
        // measured_velocity：
        // 真实速度 + [-0.01,0.01] m/s随机噪声
        // ======================================================
        measured_depth =
            current_depth
            + noise(depth_noise_amplitude);

        measured_velocity =
            velocity
            + noise(velocity_noise_amplitude);


        // ======================================================
        // 【新加3】低通滤波
        //
        // 第一次直接使用测量值进行初始化
        // ======================================================
        if (!filter_initialized)
        {
            filtered_depth =
                measured_depth;

            filtered_velocity =
                measured_velocity;

            filter_initialized = true;
        }
        else
        {
            // 深度低通滤波
            filtered_depth =
                filtered_depth
                * filter_coefficient
                +
                (1-filter_coefficient)
                * measured_depth;

            // 速度低通滤波
            filtered_velocity =
                filtered_velocity
                * filter_coefficient
                +
                (1-filter_coefficient)
                * measured_velocity;
        }


        // ======================================================
        // 【新加4】
        //
        // 为了完全保留你下面原来的controller.output这一行，
        // 这里只是临时让current_depth和velocity等于滤波值。
        //
        // 因此MPC实际使用：
        //
        // filtered_depth
        // filtered_velocity
        // ======================================================
        current_depth = filtered_depth;
        velocity = filtered_velocity;


        // ==================================================
        // 根据目标深度和Gazebo深度计算MPC输出
        // ==================================================
        double output =
        controller.output(target_depth,current_depth,velocity,dt);


        // ======================================================
        // 【新加5】
        //
        // MPC计算完成后立即恢复Gazebo真实值
        //
        // 所以后面的：
        // error
        // current_depth
        // velocity
        //
        // 仍然全部是真实状态
        // ======================================================
        current_depth = true_depth;
        velocity = true_velocity;


        // 深度误差
        double error =
            current_depth - target_depth;

  
        // ==================================================
        // 发布控制数据
        // ==================================================
        publish_double(
            output_pub_,
            output);

        publish_double(
            error_pub_,
            error);

 

        // 每1秒显示一次
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "target=%.3f, current=%.3f, error=%.3f, output=%.3f",
            target_depth,
            current_depth,
            error,
            output);

        // ==================================================
        // 写入CSV
        // ==================================================
        if (file.is_open())
        {
file
<< time << ","
<< target_depth << ","
<< current_depth << ","
<< velocity << ","
<< error << ","
<< output

// ======================================================
// 【新加】CSV后面追加噪声与滤波数据
// ======================================================
<< "," << measured_depth
<< "," << filtered_depth
<< "," << measured_velocity
<< "," << filtered_velocity

<< endl;
        // 实验过程中持续写入磁盘，防止异常退出后数据丢失
            file.flush();
        }

        step++;
    }
};


// ======================================================
// 主函数
// ======================================================
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<MPCControllerOnlyNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;};
