#include "ros/ros.h"
#include "6Dof_dyna.h"
#include "damiao.h"
#include "unistd.h"
#include <cmath>
#include <algorithm> // for std::max and std::min
#include <iostream>  // for std::cout
#include <thread>
#include <boost/thread/shared_mutex.hpp>
#include "sensor_msgs/JointState.h"
#include <vector>
#include <fstream>
#include <atomic>
#include <array>


// 硬件控制参数
constexpr double RECORD_RATE = 500.0;    // 记录频率
constexpr double RECORD_DURATION = 20.0; // 记录时长
constexpr double RETURN_SPEED = 0.5;     // 返回速度比例

// 单个关节状态数据结构
struct JointState {
    double position;    // 关节位置（弧度）
    double velocity;    // 关节速度（弧度/秒）
};

// 单帧数据（所有关节+时间戳）
struct JointFrame {
    double timestamp;   // 时间戳（相对于记录开始时间）
    std::array<JointState, 7> joints; // 6个关节状态
};

// 全局数据存储
std::vector<JointFrame> motion_buffer; // 动作数据缓冲区
std::array<double, 7> initial_positions; // 初始关节位置（弧度）

// 硬件控制对象
damiao::Motor M1(damiao::DM4310,0x01, 0x11);
damiao::Motor M2(damiao::DM4340,0x02, 0x12);
damiao::Motor M3(damiao::DM4340,0x03, 0x13);
damiao::Motor M4(damiao::DM4340,0x04, 0x14);
damiao::Motor M5(damiao::DM4310,0x05, 0x15);
damiao::Motor M6(damiao::DM4310,0x06, 0x16);
damiao::Motor M7(damiao::DMH3510,0x07, 0x17);

damiao::Motor motors[] = {M1, M2, M3, M4, M5, M6, M7};

// boost::shared_mutex serial_mutex_;  
std::shared_ptr<SerialPort> serial1;
std::shared_ptr<SerialPort> serial2;
damiao::Motor_Control dm1(serial1);
damiao::Motor_Control dm2(serial2);

void add_all_motors(){
    dm1.addMotor(&M1);
    dm1.addMotor(&M2);
    dm1.addMotor(&M3);
    dm2.addMotor(&M4);
    dm2.addMotor(&M5);
    dm2.addMotor(&M6);
    dm2.addMotor(&M7);
}

void change_motor_mode_MIT(){
    dm1.switchControlMode(M1,damiao::MIT_MODE);
    dm1.switchControlMode(M2,damiao::MIT_MODE);
    dm1.switchControlMode(M3,damiao::MIT_MODE);
    dm2.switchControlMode(M4,damiao::MIT_MODE);
    dm2.switchControlMode(M5,damiao::MIT_MODE);
    dm2.switchControlMode(M6,damiao::MIT_MODE);
}

void change_motor_mode_PosVel(){
    dm1.switchControlMode(M1,damiao::POS_VEL_MODE);
    dm1.switchControlMode(M2,damiao::POS_VEL_MODE);
    dm1.switchControlMode(M3,damiao::POS_VEL_MODE);
    dm2.switchControlMode(M4,damiao::POS_VEL_MODE);
    dm2.switchControlMode(M5,damiao::POS_VEL_MODE);
    dm2.switchControlMode(M6,damiao::POS_VEL_MODE);
}

void enable_all_motors() {
    // dm1.enable(M1);
    // dm1.enable(M2);
    // dm1.enable(M3);
    // dm2.enable(M4);
    // dm2.enable(M5);
    dm2.enable(M6);
    // dm2.enable(M7);
    ROS_INFO("All motors enabled");
}

void disable_all_motors() {
    dm1.disable(M1);
    dm1.disable(M2);
    dm1.disable(M3);
    dm2.disable(M4);
    dm2.disable(M5);
    dm2.disable(M6);
    dm2.disable(M7);
}

void refresh_motor_states() {
    dm1.refresh_motor_status(M1);
    dm1.refresh_motor_status(M2);
    dm1.refresh_motor_status(M3);
    dm2.refresh_motor_status(M4);
    dm2.refresh_motor_status(M5);
    dm2.refresh_motor_status(M6);
    dm2.refresh_motor_status(M7);
}

// 根据关节索引获取位置（示例实现）
double get_joint_position(int index) {
    switch(index) {
        case 0: return M1.Get_Position();
        case 1: return M2.Get_Position();
        case 2: return M3.Get_Position();
        case 3: return M4.Get_Position();
        case 4: return M5.Get_Position();
        case 5: return M6.Get_Position();
        case 6: return M7.Get_Position();
        default: throw std::out_of_range("Invalid joint index");
    }
}

// 根据关节索引获取速度（示例实现） 
double get_joint_velocity(int index) {
    switch(index) {
        case 0: return M1.Get_Velocity();
        case 1: return M2.Get_Velocity();
        case 2: return M3.Get_Velocity();
        case 3: return M4.Get_Velocity();
        case 4: return M5.Get_Velocity();
        case 5: return M6.Get_Velocity();
        case 6: return M7.Get_Velocity();
        default: throw std::out_of_range("Invalid joint index");
    }
}

void record_phase() {
    ROS_INFO("Start recording for %.1f seconds...", RECORD_DURATION);
    
    // 切换到MIT模式（力矩控制模式，便于被动拖动）
    change_motor_mode_MIT();

    ros::Rate rate(RECORD_RATE);
    ros::Time start_time = ros::Time::now();
    ros::Time last_print_time = start_time;
    
    while((ros::Time::now() - start_time).toSec() < RECORD_DURATION) {
        double q[7] = {0,0,0,0,0,0,0};
        double vel[7] = {0,0,0,0,0,0,0};
        double f[6] = {0,0,0,0,0,0};
        // 刷新电机状态（从硬件读取最新数据）
        refresh_motor_states();
        
        // 构建当前帧数据
        JointFrame frame;
        frame.timestamp = (ros::Time::now() - start_time).toSec();
        
        // 填充6个关节数据
        for(int i=0; i<7; ++i) {
            // 注意：这里需要根据实际电机对象调整获取方式
            frame.joints[i] = {
                .position = get_joint_position(i),  // 需要实现获取函数
                .velocity = get_joint_velocity(i)   // 需要实现获取函数
            };
            q[i] = get_joint_position(i);
            vel[i] = get_joint_velocity(i);
        }
        if(vel[0]<-0.006)  f[0] = 0.06*vel[0];
        if(vel[0]>0.006)   f[0] = 0.06*vel[0];

        if(vel[1]<-0.006)  f[1] = -0.1+0.05*vel[1];
        if(vel[1]>0.006)   f[1] = 0.1+0.05*vel[1];

        if(vel[2]<-0.006)  f[2] = -0.2+0.1*vel[2];
        if(vel[2]>0.006)   f[2] = 0.2+0.1*vel[2];

        if(vel[3]<-0.006)  f[3] = -0.30+0.2*vel[3];
        if(vel[3]>0.006)   f[3] = 0.30+0.2*vel[3];

        if(vel[4]<-0.01)  f[4] = 0.03-0.05*vel[4];
        if(vel[4]>0.01)   f[4] = -0.03-0.05*vel[4];

        // Calculate gravity compensation
        VectorXd tau = compute_gravity_compensation(q);
        tau[0] = tau[0] + f[0];
        tau[1] = tau[1] + f[1];
        tau[2] = tau[2] + f[2];
        tau[3] = tau[3] + f[3];
        tau[4] = tau[4] + f[4];
        tau[5] = tau[5] + f[5];
        //臂
        dm1.control_mit(M1, 0.0, 0.0, 0.0, 0.0, tau[0]);
        dm1.control_mit(M2, 0.0, 0.0, 0.0, 0.0, tau[1]);
        dm1.control_mit(M3, 0.0, 0.0, 0.0, 0.0, tau[2]);
        dm2.control_mit(M4, 0.0, 0.0, 0.0, 0.0, tau[3]);
        dm2.control_mit(M5, 0.0, 0.0, 0.0, 0.0, tau[4]);
        dm2.control_mit(M6, 0.0, 0.0, 0.0, 0.0, tau[5]);
        //夹爪
        dm2.control_pos_force(M7,0,0,0);

        // 每秒打印进度
        if ((ros::Time::now() - last_print_time).toSec() >= 1.0) {
            ROS_INFO("Recording progress: %.1f/%.1f seconds", 
                    frame.timestamp, RECORD_DURATION);
            last_print_time = ros::Time::now();
        }

        // 添加实时关节角度显示
        // ROS_INFO_THROTTLE(0.5,  // 每0.5秒显示一次
        //     "Joints[rad]: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f",
        //     frame.joints[0].position,
        //     frame.joints[1].position,
        //     frame.joints[2].position,
        //     frame.joints[3].position,
        //     frame.joints[4].position,
        //     frame.joints[5].position
        //     frame.joints[6].position
        // );
        ROS_INFO_THROTTLE(0.5,  // 每0.5秒显示一次
            "vel[rad]: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f",
            frame.joints[0].velocity,
            frame.joints[1].velocity,
            frame.joints[2].velocity,
            frame.joints[3].velocity,
            frame.joints[4].velocity,
            frame.joints[5].velocity,
            frame.joints[6].velocity
        );

        // 存入缓冲区
        motion_buffer.push_back(frame);
        
        // 保持400Hz记录频率
        rate.sleep();
    }
    
    ROS_INFO("Recording completed. Saved %lu frames", motion_buffer.size());
}

void save_recording(const std::string& filename) {
    std::ofstream file(filename);
    file << "timestamp,j1_pos,j1_vel,...,j6_pos,j6_vel\n";
    for (const auto& frame : motion_buffer) {
        file << frame.timestamp;
        for (const auto& joint : frame.joints) {
            file << "," << joint.position << "," << joint.velocity;
        }
        file << "\n";
    }
    ROS_INFO("Data saved to %s", filename.c_str());
}

/**
 * @brief 控制机械臂返回初始位置
 * @param slow_return 返回模式标志位
 *                    true: 慢速渐变返回（防止急动）
 *                    false: 快速直接定位返回
 */
void return_to_initial(bool slow_return) {
    ROS_INFO("Returning to initial positions...");
    
    // 切换到位置-速度控制模式（确保平稳移动）
    change_motor_mode_PosVel();

    // 控制循环参数配置
    ros::Rate rate(200);        // 控制频率200Hz（5ms周期）
    constexpr int STEPS = 800;  // 总控制步数（慢速模式时总时间=200步*5ms=1秒）

    // 分步控制循环
    for(int step=0; step<STEPS; ++step) {
        // 遍历所有6个关节
        for(int i=0; i<7; ++i) {
            // 计算位置插值比例因子
            // slow_return模式：从0到1的线性渐变（step=0时ratio=0.005, step=199时ratio=1.0）
            // 快速模式：直接使用ratio=1.0（一步到位）
            double ratio = slow_return ? 
                static_cast<double>(step+1)/STEPS : // 慢速渐变
                1.0;                                // 直接定位

            // 计算目标位置（线性插值公式）:
            // target = 初始位置 * 比例 + 当前位置 * (1 - 比例)
            // 当ratio从0→1时，目标位置从当前位置→初始位置
            double target = initial_positions[i] * ratio + 
                           motors[i].Get_Position() * (1.0 - ratio);

            // 根据关节所属控制器发送控制指令
            if(i < 3) {
                // 前3个关节使用dm1控制器
                // control_pos_vel参数：电机对象，目标位置，速度比例（0.0-1.0）
                dm1.control_pos_vel(motors[i], target, 0.3);
            } 
            else if((i >= 3) && (i < 6)) {
                // 后3个关节使用dm2控制器
                dm2.control_pos_vel(motors[i], target, 0.3);
            }
            else if (i == 6) {
                dm2.control_pos_force(motors[i],2.0,500,500);
            }
        }
        // 维持控制频率
        rate.sleep();
    }

    // dm1.control_pos_vel(motors[0], 0.0, 0.0);
    // dm1.control_pos_vel(motors[1], 0.0, 0.0);
    // dm1.control_pos_vel(motors[2], 0.0, 0.0);
    // dm2.control_pos_vel(motors[3], 0.0, 0.0);
    // dm2.control_pos_vel(motors[4], 0.0, 0.0);
    // dm2.control_pos_vel(motors[5], 0.0, 0.0);
    sleep(2);// 等待2秒
}

void replay_phase() {
    ROS_INFO("Starting motion replay...");
    
    // 切换到位置-速度模式
    // change_motor_mode_PosVel();
    // 切换到MIT模式
    change_motor_mode_MIT();

    ros::Rate rate(RECORD_RATE);
    size_t frame_counter = 0;
    
    while(frame_counter < motion_buffer.size() && ros::ok()) {
        auto& frame = motion_buffer[frame_counter];
        double current_pos[7];

        // 刷新电机状态（从硬件读取最新数据）
        refresh_motor_states();
        for(int i=0; i<7; ++i) {
                current_pos[i] = get_joint_position(i);  // 需要实现获取函数
        }

        // 添加复现位置显示
        ROS_INFO_THROTTLE(0.5, 
            "Replaying[rad]: %.5lf, %.5lf, %.5lf, %.5lf, %.5lf, %.5lf",
            current_pos[0],
            current_pos[1],
            current_pos[2],
            current_pos[3],
            current_pos[4],
            current_pos[5],
            current_pos[6]
        );

        VectorXd tau = compute_gravity_compensation(current_pos);

        // 设置每个关节的位置和速度
        for(int i=0; i<7; ++i) {
            const auto& state = frame.joints[i];
            // if(i < 3) {
            //     dm1.control_pos_vel(
            //         motors[i], 
            //         state.position,
            //         state.velocity
            //     );
            // } else {
            //     dm2.control_pos_vel(
            //         motors[i],
            //         state.position,
            //         state.velocity
            //     );
            // }
            
            if(i==0){
                dm1.control_mit(motors[i], 25.0, 1.5, state.position, state.velocity, tau[0]);
            }
            if(i==1){
                dm1.control_mit(motors[i], 52.0, 1.0, state.position, state.velocity, tau[1]);//110
            }
            if(i==2){
                dm1.control_mit(motors[i], 65.0, 1.5, state.position, state.velocity, tau[2]);//140
            }
            if(i==3){
                dm2.control_mit(motors[i], 40.0, 1.0, state.position, state.velocity, tau[3]);//120
            }
            if(i==4){
                dm2.control_mit(motors[i], 10.0, 1.0, state.position, state.velocity, tau[4]);
            }
            if(i==5){
                dm2.control_mit(motors[i], 10.0, 1.0, state.position, state.velocity, tau[5]);
            }
            if(i==6){
                //夹紧距离减小0.2保证能夹住东西
                dm2.control_pos_force(motors[i],state.position-0.2,abs(state.velocity*100),500);
            }
            
            // if(i==0) dm1.control_pos_vel(
            //     motors[i], 
            //     state.position,
            //     state.velocity
            // );
            // if(i==1) dm1.control_pos_vel(
            //     motors[i], 
            //     state.position,
            //     state.velocity
            // );
            // if(i==2) dm1.control_pos_vel(
            //     motors[i], 
            //     state.position,
            //     state.velocity
            // );
            // if(i==3) dm2.control_pos_vel(
            //     motors[i], 
            //     state.position,
            //     state.velocity
            // );
            // if(i==4) dm2.control_pos_vel(
            //     motors[i], 
            //     state.position,
            //     state.velocity
            // );
            // if(i==5) dm2.control_pos_vel(
            //     motors[i], 
            //     state.position,
            //     state.velocity
            // );
        }
        ++frame_counter;
        rate.sleep();
    }
    
    ROS_INFO("Replay completed");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "teach");
    ros::NodeHandle nh;  // 添加ROS节点句柄初始化

    serial1 = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
    serial2 = std::make_shared<SerialPort>("/dev/ttyACM1", B921600);
    dm1 = damiao::Motor_Control(serial1);
    dm2 = damiao::Motor_Control(serial2);
    
    add_all_motors();
    enable_all_motors();
    
    // 获取初始位置
    // refresh_motor_states();
    // for(int i=0; i<6; ++i) {
    //     initial_positions[i] = motors[i].Get_Position();
    // }
    initial_positions ={0,0,0,0,0,0,2.0};


    // 主执行流程
    return_to_initial(true); // 阶段：慢速回初始
    record_phase();        // 阶段1：记录动作
    return_to_initial(true); // 阶段2：慢速回初始
    replay_phase();        // 阶段3：复现动作
    return_to_initial(true);// 阶段4：快速回初始
    save_recording("motion_data.csv");
    
    disable_all_motors();
    return 0;
}