#include "ros/ros.h"
#include "damiao.h"
#include "unistd.h"
#include <cmath>
#include <algorithm> // for std::max and std::min
#include <iostream>  // for std::cout


damiao::Motor M1(damiao::DM4310,0x01, 0x11);
damiao::Motor M2(damiao::DM4340,0x02, 0x12);
damiao::Motor M3(damiao::DM4340,0x03, 0x13);
damiao::Motor M4(damiao::DM4340,0x04, 0x14);
damiao::Motor M5(damiao::DM4310,0x05, 0x15);
damiao::Motor M6(damiao::DM4310,0x06, 0x16);
damiao::Motor M7(damiao::DMH3510,0x07, 0x17);

std::shared_ptr<SerialPort> serial1;
std::shared_ptr<SerialPort> serial2;
damiao::Motor_Control dm1(serial1);
damiao::Motor_Control dm2(serial2);


int main(int argc, char  *argv[])
{
    ros::init(argc,argv,"changeMode");
    ros::NodeHandle nh;
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
    
    // dm1.switchControlMode(M1,damiao::POS_VEL_MODE);
    // dm1.switchControlMode(M2,damiao::POS_VEL_MODE);
    // dm1.switchControlMode(M3,damiao::POS_VEL_MODE);
    // dm2.switchControlMode(M4,damiao::POS_VEL_MODE);
    // dm2.switchControlMode(M5,damiao::POS_VEL_MODE);
    // dm2.switchControlMode(M6,damiao::POS_VEL_MODE);


    return 0;
}
