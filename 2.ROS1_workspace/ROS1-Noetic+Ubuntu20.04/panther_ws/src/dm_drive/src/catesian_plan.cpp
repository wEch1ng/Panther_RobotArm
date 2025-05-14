#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <vector>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// 安全运动参数
constexpr double CIRCLE_RADIUS = 0.075;      // 圆周半径(米)
constexpr int POINTS_PER_CIRCLE = 72;         // 每圈路径点数（提高平滑度）
constexpr int TOTAL_CIRCLES = 5;              // 总圈数
constexpr double EXECUTION_VELOCITY = 0.6;   // 降低执行速度（安全第一）
constexpr double STABLE_DELAY = 2.0;          // 各阶段间的稳定等待时间

// 安全执行函数（带状态检查）
bool safeExecute(moveit::planning_interface::MoveGroupInterface& group, 
                const moveit::planning_interface::MoveGroupInterface::Plan& plan) 
{
    if(!group.execute(plan)) {
        ROS_ERROR("Execution failed!");
        return false;
    }
    ros::Duration(STABLE_DELAY).sleep(); // 确保完全停止
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "safe_circular_motion");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // 初始化MoveGroup（严格参数设置）
    moveit::planning_interface::MoveGroupInterface arm("arm");
    arm.setMaxVelocityScalingFactor(EXECUTION_VELOCITY);
    arm.setPlanningTime(20.0);    // 大幅增加规划时间
    arm.setNumPlanningAttempts(10); // 增加尝试次数

    // ===== 1. 安全移动到近端点 =====
    ROS_INFO("=== plan1: move to the start ===");
    geometry_msgs::Pose start_pose;
    start_pose.position.x = 0.075;//0.086361;
    start_pose.position.y = 0.0;
    start_pose.position.z = 0.15;
    start_pose.orientation.x = 0.704403;
    start_pose.orientation.y = 0.709797;
    start_pose.orientation.z = -0.000571;
    start_pose.orientation.w = 0.001980;

    
    arm.setPoseTarget(start_pose);
    moveit::planning_interface::MoveGroupInterface::Plan approach_plan;
    
    if(!arm.plan(approach_plan) || !safeExecute(arm, approach_plan)) {
        ROS_ERROR("go to the start point fail！stop!！");
        return -1;
    }

    // ===== 2. 生成单圈连续轨迹 ===== 
    ROS_INFO("=== plan2: generate arc trajectory ===");
    geometry_msgs::Point center;
    center.x = 0.15;//(0.086361 + 0.213433)/2.0
    center.y = 0.0;
    center.z = 0.15;

    std::vector<geometry_msgs::Pose> circle_waypoints;
    for(int i = 0; i <= POINTS_PER_CIRCLE; ++i) {
        double angle = 2 * M_PI * i / POINTS_PER_CIRCLE;
        geometry_msgs::Pose pose;
        pose.position.x = center.x + CIRCLE_RADIUS * cos(angle - M_PI);
        pose.position.y = center.y + CIRCLE_RADIUS * sin(angle - M_PI); 
        pose.position.z = center.z;
        pose.orientation = start_pose.orientation;
        circle_waypoints.push_back(pose);
    }

    // ===== 3. 执行1圈连续运动 =====
    ROS_INFO("=== plan3: go to %d arc  motion===", TOTAL_CIRCLES);
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = arm.computeCartesianPath(
        circle_waypoints, 0.01,  // 更精细的步长
        0.0, trajectory, true);

    if(fraction < 0.95) {
        ROS_ERROR("plan the arc trajectory fail (finish: %.1f%%)", fraction*100);
        return -1;
    }
    if(fraction == 1.0){
        arm.execute(trajectory);
    }
        ros::Duration(STABLE_DELAY).sleep();  // 额外延时确保停止

    // ===== 4. 确保完全停止后返回home =====
    ROS_INFO("=== plan4: go to the home  ===");
    ros::Duration(STABLE_DELAY*2).sleep();  // 额外延时确保停止
    
    arm.setNamedTarget("home");
    moveit::planning_interface::MoveGroupInterface::Plan home_plan;
    if(arm.plan(home_plan)) {
        safeExecute(arm, home_plan);
    } else {
        ROS_ERROR("Home plan fail！please required！");
    }

    ROS_INFO("==== move finish！%darc finish ====", TOTAL_CIRCLES);
    ros::shutdown();
    return 0;
}