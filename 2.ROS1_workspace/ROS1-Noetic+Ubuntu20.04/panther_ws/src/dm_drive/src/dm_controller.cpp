#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>
#include <shared_mutex>
#include <mutex>
#include <boost/thread/shared_mutex.hpp>
#include <queue>
#include <thread>  
#include <std_msgs/Float32MultiArray.h>
#include <iostream>
#include <moveit_msgs/RobotTrajectory.h>
#include <vector>
#include <trajectory_msgs/JointTrajectory.h>
#include <ros/duration.h>
#include "std_msgs/String.h"
#include "damiao.h"
#include "unistd.h"
#include <cmath>
#include <algorithm> // for std::max and std::min
#include <iostream>  // for std::cout
#include <array>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include "6Dof_dyna.h"
#include <Eigen/Core>
#include <algorithm>

using namespace Eigen;
using namespace std;

// 控制参数
const double POSITION_TOLERANCE = 0.06;   // 弧度
const double TIME_TOLERANCE = 0.1;        // 秒
const double MAX_DELAY_COMPENSATION = 0.5;// 最大延时补偿

// 自定义三次样条插值类
class CubicSpline {
    private:
        std::vector<double> x_, y_;  // 原始数据点
        std::vector<double> b_, c_, d_;  // 样条系数
    
    public:
        // 设置数据点
        void set_points(const std::vector<double>& x, const std::vector<double>& y) {
            if (x.size() != y.size()) {
                throw std::invalid_argument("x and y must have the same size");
            }
            if (x.size() < 2) {
                throw std::invalid_argument("At least 2 points are required");
            }
    
            x_ = x;
            y_ = y;
            int n = x.size();
            b_.resize(n);
            c_.resize(n);
            d_.resize(n);
    
            // 计算系数
            std::vector<double> h(n - 1), alpha(n - 1);
            for (int i = 0; i < n - 1; ++i) {
                h[i] = x[i + 1] - x[i];
                alpha[i] = (y[i + 1] - y[i]) / h[i];
            }
    
            std::vector<double> l(n), mu(n), z(n);
            l[0] = 1;
            for (int i = 1; i < n - 1; ++i) {
                l[i] = 2 * (x[i + 1] - x[i - 1]) - h[i - 1] * mu[i - 1];
                mu[i] = h[i] / l[i];
                z[i] = (alpha[i] - alpha[i - 1] - h[i - 1] * z[i - 1]) / l[i];
            }
    
            l[n - 1] = 1;
            z[n - 1] = 0;
            c_[n - 1] = 0;
    
            for (int j = n - 2; j >= 0; --j) {
                c_[j] = z[j] - mu[j] * c_[j + 1];
                b_[j] = (y[j + 1] - y[j]) / h[j] - h[j] * (c_[j + 1] + 2 * c_[j]) / 3;
                d_[j] = (c_[j + 1] - c_[j]) / (3 * h[j]);
            }
        }
    
        // 插值计算
        double operator()(double x) const {
            auto it = std::upper_bound(x_.begin(), x_.end(), x);
            int idx = std::max(int(it - x_.begin()) - 1, 0);
            double dx = x - x_[idx];
            return y_[idx] + b_[idx] * dx + c_[idx] * dx * dx + d_[idx] * dx * dx * dx;
        }

        double derivative(double x) const {
            auto it = std::upper_bound(x_.begin(), x_.end(), x);
            int idx = std::max(int(it - x_.begin()) - 1, 0);
            double dx = x - x_[idx];
            return b_[idx] + 2 * c_[idx] * dx + 3 * d_[idx] * dx * dx;
        }
    };

class HybridTrigger {
    private:
        bool use_time_based_ = true;    // 初始基于时间触发
        double position_tolerance_ = 0.05;
        double max_time_error_ = 0.1;    // 最大时间误差
        
    public:
        bool shouldSendNextPoint(const ros::Time& expected_time,
                                const std::vector<double>& current_pos,
                                const std::vector<double>& target_pos) 
        {
            // 时间误差超过阈值时切换为位置触发
            double time_error = fabs((ros::Time::now() - expected_time).toSec());
            if(time_error > max_time_error_){
                use_time_based_ = false;
            }
    
            // 位置接近目标时切换回时间触发
            bool position_ok = checkPosition(current_pos, target_pos);
            if(position_ok && !use_time_based_){
                use_time_based_ = true;
            }
    
            return use_time_based_ ? (ros::Time::now() >= expected_time) : position_ok;
        }
        bool checkPosition(const std::vector<double>& target, const std::vector<double>& actual) const {
            if (target.size() != actual.size()) {
                ROS_ERROR("Target and actual positions have different sizes!");
            return false;
            }

            for (size_t i = 0; i < target.size(); ++i) {
                if (fabs(target[i] - actual[i]) > POSITION_TOLERANCE) {
                    return false;
                }
            }
            return true;
        }
};

class DmController {
private:
    // 硬件接口
    // 新增串口操作锁
    boost::shared_mutex serial_mutex_;  
    damiao::Motor M1, M2, M3, M4, M5, M6,M7;
    std::shared_ptr<SerialPort> serial1;
    std::shared_ptr<SerialPort> serial2;
    damiao::Motor_Control dm1;
    damiao::Motor_Control dm2;
    std::vector<CubicSpline> splines_;

    // ROS 接口
    ros::Publisher joint_state_pub_;
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> gripper_as_;
    
    // 共享数据
    boost::shared_mutex data_mutex_;
    std::queue<trajectory_msgs::JointTrajectoryPoint> trajectory_queue_;
    sensor_msgs::JointState current_joint_state_;
    bool trajectory_running_ = false;

public:
    DmController(ros::NodeHandle& nh) : 
        as_(nh, "/arm_controller/follow_joint_trajectory", 
            boost::bind(&DmController::executeCB, this, _1), false),
        gripper_as_(nh, "/gripper_controller/follow_joint_trajectory",
            boost::bind(&DmController::gripperExecuteCB, this, _1),false),
        M1(damiao::DM4310,0x01,0x11), M2(damiao::DM4340,0x02,0x12),
        M3(damiao::DM4340,0x03,0x13), M4(damiao::DM4340,0x04,0x14),
        M5(damiao::DM4310,0x05,0x15), M6(damiao::DM4310,0x06,0x16),
        M7(damiao::DMH3510,0x07,0x17)
    {
        // 初始化硬件
        serial1 = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
        serial2 = std::make_shared<SerialPort>("/dev/ttyACM1", B921600);
        dm1 = damiao::Motor_Control(serial1);
        dm2 = damiao::Motor_Control(serial2);

        HybridTrigger trigger;

        addAllMotor();

        ROS_INFO("Motor enabled success!!");
        // 初始化ROS
        joint_state_pub_ = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
        as_.start();
        gripper_as_.start();

        // 启动状态发布线程
        std::thread(&DmController::publishJointStates, this).detach();
    }

    // 关节状态发布线程
    void publishJointStates() {
        ros::Rate rate(400);
        while (ros::ok()) {
            sensor_msgs::JointState js;
            {
                boost::unique_lock<boost::shared_mutex> lock(data_mutex_);
                js = current_joint_state_;
            }
            js.header.stamp = ros::Time::now();
            joint_state_pub_.publish(js);
            rate.sleep();
        }
    }
    // 轨迹执行回调
    void executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal) {
        // 1. 检查轨迹有效性
        if (goal->trajectory.points.empty()) {
            ROS_ERROR("Received empty trajectory!");
            as_.setAborted();
            return;
        }

        // 2. 提取轨迹数据
        const auto& trajectory = goal->trajectory;
        int n_joints = trajectory.joint_names.size();
        int n_points = trajectory.points.size();

        // 3. 初始化样条对象
        splines_.resize(n_joints);
        std::vector<double> times(n_points);
        for (int i = 0; i < n_points; ++i) {
            times[i] = trajectory.points[i].time_from_start.toSec();
        }

        // 4. 为每个关节设置样条
        for (int j = 0; j < n_joints; ++j) {
            std::vector<double> positions(n_points);
            for (int i = 0; i < n_points; ++i) {
                positions[i] = trajectory.points[i].positions[j];
            }
            splines_[j].set_points(times, positions);
        }

        // 5. 插值执行
        ros::Time start_time = ros::Time::now();
        double control_rate = 400;  // 控制频率 (Hz)
        ros::Rate rate(control_rate);

        while (ros::ok()) {
            // 5.1 计算当前时间
            double elapsed = (ros::Time::now() - start_time).toSec();

            // 5.2 检查是否完成
            if (elapsed > times.back()) {
                break;
            }

            // 5.3 插值计算目标位置
            trajectory_msgs::JointTrajectoryPoint point;
            point.positions.resize(n_joints);
            point.velocities.resize(n_joints);
            for (int j = 0; j < n_joints; ++j) {
                point.positions[j] = splines_[j](elapsed);
                point.velocities[j] = splines_[j].derivative(elapsed);
            }
            // 5.4 发送目标到电机
            sendToMotor(point);

            // 5.5 等待下一周期
            rate.sleep();
        }

        // 6. 返回成功
        as_.setSucceeded();
    }

// 新增夹爪执行回调
void gripperExecuteCB(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal) {
    // 1. 轨迹有效性检查
    if (goal->trajectory.points.empty()) {
        ROS_ERROR("Gripper: Empty trajectory received!");
        gripper_as_.setAborted();
        return;
    }

    // 2. 检查关节名称
    if (goal->trajectory.joint_names[0] != "gripper_1_joint") {
        ROS_ERROR("Invalid joint name for gripper!");
        gripper_as_.setAborted();
        return;
    }

    // 3. 插值处理（简单线性插值）
    const auto& points = goal->trajectory.points;
    ros::Time start_time = ros::Time::now();
    
        // 4. 检查是否被抢占
        if (gripper_as_.isPreemptRequested()) {
            gripper_as_.setPreempted();
            return;
        }

        // 直接取最后一个点
        const auto& last_point = goal->trajectory.points.back();

        // 5. 计算目标位置
        double target_pos = (last_point.positions[0]/0.04*4.0)-2.0;
        // double target_vel = point.velocities[0]*10400;
        
        ROS_INFO("POS:%f",target_pos);

        // 6. 发送到夹爪电机
        {
            boost::unique_lock<boost::shared_mutex> lock(serial_mutex_);
            // 假设使用位置控制模式
            dm2.control_pos_force(M7, target_pos,500,700);
        }

        // 7. 等待执行时间
        ros::Duration(2).sleep();  // 根据实际需求调整    

    // 8. 返回成功
    gripper_as_.setSucceeded();
}

    // 发送单个轨迹点到电机
    void sendToMotor(const trajectory_msgs::JointTrajectoryPoint& point) {
        if (point.positions.size() < 6 || point.velocities.size() < 6) {
            ROS_ERROR("Trajectory point data is invalid!");
            return;
        }
        std::vector<double> current_pos;
        {
            boost::shared_lock<boost::shared_mutex> lock(data_mutex_);
            current_pos = current_joint_state_.position;
        }
        double q[6] = {current_pos[0],current_pos[1], current_pos[2], current_pos[3], current_pos[4], current_pos[5]};
        // 计算重力补偿力矩
        // VectorXd tau = compute_gravity_compensation(q);
        {
            boost::unique_lock<boost::shared_mutex> serial_lock(serial_mutex_);
            // dm1.control_mit(M1, 25.0, 1.5, point.positions[0], point.velocities[0], tau[0]);
            // dm1.control_mit(M2, 52.0, 1.0, point.positions[1], point.velocities[1], tau[1]);//110
            // dm1.control_mit(M3, 65.0, 1.5, point.positions[2], point.velocities[2], tau[2]);//140
            // dm2.control_mit(M4, 40.0, 1.0, point.positions[3], point.velocities[3], tau[3]);//120
            // dm2.control_mit(M5, 10.0, 1.0, point.positions[4], point.velocities[4], tau[4]);
            // dm2.control_mit(M6, 10.0, 1.0, point.positions[5], point.velocities[5], tau[5]);
            dm1.control_pos_vel(M1,point.positions[0], point.velocities[0]);
            dm1.control_pos_vel(M2,point.positions[1], point.velocities[1]);
            dm1.control_pos_vel(M3,point.positions[2], point.velocities[2]);
            dm2.control_pos_vel(M4,point.positions[3], point.velocities[3]);
            dm2.control_pos_vel(M5,point.positions[4], point.velocities[4]);
            dm2.control_pos_vel(M6,point.positions[5], point.velocities[5]);
        }
        
        // ROS_INFO("pos:[%f],[%f],[%f],[%f],[%f],[%f]",point.positions[0], point.positions[1], point.positions[2], point.positions[3], point.positions[4], point.positions[5]);
        // ROS_INFO("vel:[%f],[%f],[%f],[%f],[%f],[%f]",point.velocities[0], point.velocities[1], point.velocities[2], point.velocities[3], point.velocities[4], point.velocities[5]);
    }

    //轨迹插值预处理
void interpolateTrajectory(const trajectory_msgs::JointTrajectory& input,trajectory_msgs::JointTrajectory& output,double control_interval) {
    // 提取时间序列
    std::vector<double> times(input.points.size());
    for(size_t i=0; i<input.points.size(); ++i){
        times[i] = input.points[i].time_from_start.toSec();
    }

    // 为每个关节创建样条曲线
    std::vector<CubicSpline> splines(input.joint_names.size());
    for(size_t j=0; j<input.joint_names.size(); ++j){
        std::vector<double> positions(input.points.size());
        for(size_t i=0; i<input.points.size(); ++i){
            positions[i] = input.points[i].positions[j];
        }
        splines[j].set_points(times, positions);
    }

    // 生成等间隔轨迹点
    double t = 0;
    while(t <= times.back()){
        trajectory_msgs::JointTrajectoryPoint point;
        point.time_from_start = ros::Duration(t);
        for(size_t j=0; j<splines.size(); ++j){
            point.positions.push_back(splines[j](t));
            point.velocities.push_back(splines[j].derivative(t));
        }
        output.points.push_back(point);
        t += control_interval;
    }
}

    void adjustTimeStamps(trajectory_msgs::JointTrajectory& trajectory, double delay_compensation) {
        for(auto& point : trajectory.points){
            point.time_from_start += ros::Duration(delay_compensation);
        }
    }
    // 位置检查
    bool checkPosition(const std::vector<double>& target, 
                      const std::vector<double>& actual) {
        for (size_t i = 0; i < target.size(); ++i) {
            if (fabs(target[i] - actual[i]) > POSITION_TOLERANCE)
                return false;
        }
        return true;
    }

    void refreshAllMotorStatus(){
        {
            boost::unique_lock<boost::shared_mutex> serial_lock(serial_mutex_);
            dm1.refresh_motor_status(M1);
            dm1.refresh_motor_status(M2);
            dm1.refresh_motor_status(M3);
            dm2.refresh_motor_status(M4);
            dm2.refresh_motor_status(M5);
            dm2.refresh_motor_status(M6);
            dm2.refresh_motor_status(M7);
        }
    }

    // 更新关节状态（由硬件线程调用）
    void updateJointStates() {
        double current_pos[7];
        double current_vel[7];
        
        sensor_msgs::JointState js;
        js.name = {"joint1","joint2","joint3","joint4","joint5","joint6","gripper_1_joint"};
        refreshAllMotorStatus();
        
        {
            boost::unique_lock<boost::shared_mutex> serial_lock(serial_mutex_);
            current_pos[0] = M1.Get_Position();current_pos[1] = M2.Get_Position();current_pos[2] = M3.Get_Position();
            current_pos[3] = M4.Get_Position();current_pos[4] = M5.Get_Position();current_pos[5] = M6.Get_Position();
            current_pos[6] = (M7.Get_Position()+2.0)*0.04/4.0;
            current_vel[0] = M1.Get_Velocity();current_vel[1] = M2.Get_Velocity();current_vel[2] = M3.Get_Velocity();
            current_vel[3] = M4.Get_Velocity();current_vel[4] = M5.Get_Velocity();current_vel[5] = M6.Get_Velocity();
            current_vel[6] = (M7.Get_Velocity()+2.0)*0.04/4.0;
        }

            js.position = {current_pos[0], current_pos[1], current_pos[2], current_pos[3], current_pos[4], current_pos[5],current_pos[6]};
            js.velocity = { current_vel[0], current_vel[1], current_vel[2], current_vel[3], current_vel[4], current_vel[5],current_vel[6]};
        {
            boost::unique_lock<boost::shared_mutex>  lock(data_mutex_);
            current_joint_state_ = js;
        }
    }

    void addAllMotor() {
        boost::unique_lock<boost::shared_mutex> serial_lock(serial_mutex_);
        dm1.addMotor(&M1);
        dm1.addMotor(&M2);
        dm1.addMotor(&M3);
        dm2.addMotor(&M4);
        dm2.addMotor(&M5);
        dm2.addMotor(&M6);
        dm2.addMotor(&M7);
    }

    void enableAllMotors() {
        {
            boost::unique_lock<boost::shared_mutex> serial_lock(serial_mutex_);
            dm1.enable(M1);
            dm1.enable(M2);
            dm1.enable(M3);
            dm2.enable(M4);
            dm2.enable(M5);
            dm2.enable(M6);
            dm2.enable(M7);
        }
    }

    void disableAllMotors() {
        {
            boost::unique_lock<boost::shared_mutex> serial_lock(serial_mutex_);
            dm1.disable(M1);
            dm1.disable(M2);
            dm1.disable(M3);
            dm2.disable(M4);
            dm2.disable(M5);
            dm2.disable(M6);
            dm2.disable(M7);
        }
    }

    void switchControlMode() {
        {
            boost::unique_lock<boost::shared_mutex> serial_lock(serial_mutex_);
            dm1.switchControlMode(M1,damiao::POS_VEL_MODE);
            dm1.switchControlMode(M2,damiao::POS_VEL_MODE);
            dm1.switchControlMode(M3,damiao::POS_VEL_MODE);
            dm2.switchControlMode(M4,damiao::POS_VEL_MODE);
            dm2.switchControlMode(M5,damiao::POS_VEL_MODE);
            dm2.switchControlMode(M6,damiao::POS_VEL_MODE);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "dm_controller");
    ros::NodeHandle nh;
    
    DmController controller(nh);

    //切换位置速度模式
    controller.switchControlMode();
    // 启用所有电机
    controller.enableAllMotors();
    // 硬件状态更新线程
    std::thread hardware_thread([&](){
        ros::Rate update_rate(400);
        while (ros::ok()) {
            controller.updateJointStates();
            update_rate.sleep();
        }
    });

    ros::AsyncSpinner spinner(4);//创建3个线程
    spinner.start();
    hardware_thread.join();
    
    controller.disableAllMotors();

    return 0;
}