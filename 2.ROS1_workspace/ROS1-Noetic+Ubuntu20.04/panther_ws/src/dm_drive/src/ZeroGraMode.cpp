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

damiao::Motor M1(damiao::DM4310,0x01, 0x11);
damiao::Motor M2(damiao::DM4340,0x02, 0x12);
damiao::Motor M3(damiao::DM4340,0x03, 0x13);
damiao::Motor M4(damiao::DM4340,0x04, 0x14);
damiao::Motor M5(damiao::DM4310,0x05, 0x15);
damiao::Motor M6(damiao::DM4310,0x06, 0x16);
damiao::Motor M7(damiao::DMH3510,0x07, 0x17);

// boost::shared_mutex serial_mutex_;  
std::shared_ptr<SerialPort> serial1;
std::shared_ptr<SerialPort> serial2;
damiao::Motor_Control dm1(serial1);
damiao::Motor_Control dm2(serial2);

boost::shared_mutex data_mutex_;
double q[6];
double vel[6];
double f[6];
VectorXd tau = VectorXd::Zero(6);

// Function prototypes
void gravityCompensationThread(ros::Rate rate);
void jointStatePublisherThread(ros::Rate rate, ros::Publisher* joint_state_pub);

int main(int argc, char  *argv[])
{
    ros::init(argc,argv,"ZeroGraMode");
    ros::NodeHandle nh;
    ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
    sensor_msgs::JointState joint_state;
    serial1 = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
    serial2 = std::make_shared<SerialPort>("/dev/ttyACM1", B921600);
    dm1 = damiao::Motor_Control(serial1);
    dm2 = damiao::Motor_Control(serial2);
    dm1.addMotor(&M1);
    dm1.addMotor(&M2);
    dm1.addMotor(&M3);
    dm2.addMotor(&M4);
    dm2.addMotor(&M5);
    dm2.addMotor(&M6);
    dm2.addMotor(&M7);

    dm1.switchControlMode(M1,damiao::MIT_MODE);
    dm1.switchControlMode(M2,damiao::MIT_MODE);
    dm1.switchControlMode(M3,damiao::MIT_MODE);
    dm2.switchControlMode(M4,damiao::MIT_MODE);
    dm2.switchControlMode(M5,damiao::MIT_MODE);
    dm2.switchControlMode(M6,damiao::MIT_MODE);

    joint_state.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6","gripper_1_joint"};
    joint_state.position = {0, 0, 0, 0, 0, 0, 0};

    dm1.enable(M1);
    usleep(500);
    dm1.enable(M2);
    usleep(500);
    dm1.enable(M3);
    usleep(500);
    dm2.enable(M4);
    usleep(500);
    dm2.enable(M5);
    usleep(500);
    dm2.enable(M6);
    usleep(500);

    // Create threads with different rates
    ros::Rate gc_rate(500); // 500Hz for gravity compensation
    ros::Rate js_rate(100); // 100Hz for joint state publishing
    
    std::thread gc_thread(gravityCompensationThread, gc_rate);
    std::thread js_thread(jointStatePublisherThread, js_rate, &joint_state_pub);

    // Wait for threads to finish (they won't in this case unless ROS shuts down)
    gc_thread.join();
    js_thread.join();

    dm1.disable(M1);
    dm1.disable(M2);
    dm1.disable(M3);
    dm2.disable(M4);
    dm2.disable(M5);
    dm2.disable(M6);
    return 0;
}

void gravityCompensationThread(ros::Rate rate) {
    while(ros::ok()) {
        ros::Time current_time = ros::Time::now();
        
        // Refresh motor status
        dm1.refresh_motor_status(M1);
        dm1.refresh_motor_status(M2);
        dm1.refresh_motor_status(M3);
        dm2.refresh_motor_status(M4);
        dm2.refresh_motor_status(M5);
        dm2.refresh_motor_status(M6);
        dm2.refresh_motor_status(M7);

        // Update joint positions
        {
            boost::unique_lock<boost::shared_mutex> lock(data_mutex_);
            q[0] = M1.Get_Position();
            q[1] = M2.Get_Position();
            q[2] = M3.Get_Position();
            q[3] = M4.Get_Position();
            q[4] = M5.Get_Position();
            q[5] = M6.Get_Position();
            
            vel[0] = M1.Get_Velocity();
            vel[1] = M2.Get_Velocity();
            vel[2] = M3.Get_Velocity();
            vel[3] = M4.Get_Velocity();
            vel[4] = M5.Get_Velocity();
            vel[5] = M6.Get_Velocity();
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

        // Send torque commands
        dm1.control_mit(M1, 0.0, 0.0, 0.0, 0.0, tau[0]);
        dm1.control_mit(M2, 0.0, 0.0, 0.0, 0.0, tau[1]);
        dm1.control_mit(M3, 0.0, 0.0, 0.0, 0.0, tau[2]);
        dm2.control_mit(M4, 0.0, 0.0, 0.0, 0.0, tau[3]);
        dm2.control_mit(M5, 0.0, 0.0, 0.0, 0.0, tau[4]);
        dm2.control_mit(M6, 0.0, 0.0, 0.0, 0.0, tau[5]);

        ros::Time current_time1 = ros::Time::now();
        double time_diff = (current_time1 - current_time).toSec();
        ROS_INFO("GC Fre: %f", 1/time_diff);
        ROS_INFO("t:[%f] [%f] [%f] [%f] [%f] [%f]", tau[0], tau[1], tau[2], tau[3], tau[4], tau[5]);

        rate.sleep();
    }
}

void jointStatePublisherThread(ros::Rate rate, ros::Publisher* joint_state_pub) {
    sensor_msgs::JointState joint_state;
    joint_state.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6","gripper_1_joint"};
    joint_state.position.resize(7);

    while(ros::ok()) {
        // Get current positions with thread-safe access
        double positions[7];
        {
            boost::shared_lock<boost::shared_mutex> lock(data_mutex_);
            positions[0] = M1.Get_Position();
            positions[1] = M2.Get_Position();
            positions[2] = M3.Get_Position();
            positions[3] = M4.Get_Position();
            positions[4] = M5.Get_Position();
            positions[5] = M6.Get_Position();
            positions[6] = (M7.Get_Position()+2.0)/4.0*0.04;
        }

        // Update and publish joint state
        for(int i = 0; i < 7; i++) {
            joint_state.position[i] = positions[i];
        }
        joint_state.header.stamp = ros::Time::now();
        joint_state_pub->publish(joint_state);

        rate.sleep();
    }
}