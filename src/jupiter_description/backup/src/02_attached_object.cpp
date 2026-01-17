#include <iostream>
#include "ros/ros.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/LinearMath/Quaternion.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

using namespace std;

int main(int argc, char **argv) {
    //ROS节点初始化 Initialize ROS node 
    ros::init(argc, argv, "attached_object_cpp");
    //创建节点句柄  Create node handle
    ros::NodeHandle n;
    // 设置线程 Set thread
    ros::AsyncSpinner spinner(1);
    // 开启线程 Start thread
    spinner.start();
    // 初始化机械臂 Initialize the robotic arm
    moveit::planning_interface::MoveGroupInterface jupiter("arm");
	// 允许重新规划 Allow replanning
    jupiter.allowReplanning(true);
    // 规划的时间(单位：秒) Planning time (unit: second)
    jupiter.setPlanningTime(5);
    // 设置规划尝试次数 Set the number of planning attempts
    jupiter.setNumPlanningAttempts(10);
    // 设置允许目标姿态误差(单位：米) Set the allowable target attitude error (unit: meter)
    jupiter.setGoalPositionTolerance(0.01);
    // 设置允许目标位置误差(单位：弧度) Set the allowable target position error (unit: radians)
    jupiter.setGoalOrientationTolerance(0.01);
    // 设置最大速度 Set maximum speed
    jupiter.setMaxVelocityScalingFactor(1.0);
    // 设置最大加速度 Set maximum acceleration
    jupiter.setMaxAccelerationScalingFactor(1.0);
    //设置目标点 Set the target point
    jupiter.setNamedTarget("pose1");
    //开始移动 Start moving
    jupiter.move();
    sleep(0.1);
    // 获取机器人正在计划的框架的名称 Get the name of the framework the bot is planning
    string frame = jupiter.getPlanningFrame();
    // 创建场景实例 Create scenario instance
    moveit::planning_interface::PlanningSceneInterface scene;
    /////////////////////////添加障碍物 Add obstacles ///////////////////////
    vector<string> object_ids;
    scene.removeCollisionObjects(object_ids);
    // 创建检测对象容器 Create inspection object container
    vector<moveit_msgs::CollisionObject> objects;
    // 创建碰撞检测对象 Create collision detection object
    moveit_msgs::CollisionObject obj;
    // 设置障碍物的id Set the id of the obstacle
    obj.id = "obj";
    object_ids.push_back(obj.id);
    // 障碍物的状态 The state of the obstacle
    obj.operation = obj.ADD;
    // 设置障碍物的头信息 Set the header information of the obstacle
    obj.header.frame_id = frame;
    shape_msgs::SolidPrimitive primitive;
    shape_msgs::SolidPrimitive floor;
    // 设置障碍物类型 Set type of obstacle
    primitive.type = primitive.BOX;
    floor.type = primitive.BOX;
    // 设置障碍物维度 Set dimensions of obstacle
    primitive.dimensions.resize(3);
    floor.dimensions.resize(3);
    // 设置障碍物的长宽高 Set length width hight of obstacle
    primitive.dimensions[0] = 0.2;
    primitive.dimensions[1] = 0.1;
    primitive.dimensions[2] = 0.01;
    obj.primitives.push_back(primitive);
    floor.dimensions[0] = 1;
    floor.dimensions[1] = 0.5;
    floor.dimensions[2] = 0.01;
    obj.primitives.push_back(floor);
    geometry_msgs::Pose pose;
    geometry_msgs::Pose pose1;
    // 设置障碍物的位置信息[x,y,z]  Set locatio info
    pose.position.x = 0.36;
    pose.position.y = 0;
    pose.position.z = 0.125;
    // 设置障碍物的姿态信息  Set quaternion info
    tf::Quaternion quaternion;
    // R,P,Y的单位是角度 R,P,Y U unit is angle
    double Roll = 0.0;
    double Pitch = 0.0;
    double Yaw = 90.0;
    // 将RPY转换成四元数  Convert RPY to quaternion
    quaternion.setRPY(Roll * M_PI / 180, Pitch * M_PI / 180, Yaw * M_PI / 180);
    pose.orientation.x = quaternion.x();
    pose.orientation.y = quaternion.y();
    pose.orientation.z = quaternion.z();
    pose.orientation.w = quaternion.w();
    // 设置障碍物的位姿信息 Set pose info of obstacle
    obj.primitive_poses.push_back(pose);
    obj.primitive_poses.push_back(pose1);
    objects.push_back(obj);
    /////////////////////////设置障碍物的颜色Set color of obstacle///////////////////////
    // 创建检测对象的颜色容器 Create a color container for the detection object
    std::vector<moveit_msgs::ObjectColor> colors;
    // 创建颜色实例 Create color instance
    moveit_msgs::ObjectColor color;
    // 添加需要设置颜色的id  Add id that need to set color
    color.id = "obj";
    // 设置RGBA值,范围[0~1]  Set RGBA value, range [0~1]
    color.color.r = 0;
    color.color.g = 1.0;
    color.color.b = 0;
    color.color.a = 0.5;
    colors.push_back(color);
    // 将设置的信息添加到场景中  Add the set information to the scene
    scene.applyCollisionObjects(objects, colors);
    // 设置目标点  Set the target point
    jupiter.setNamedTarget("pose1");
    // 开始移动 Start moving
    jupiter.move();
    sleep(0.1);
    vector<double> target_joints1 = jupiter.getCurrentJointValues();
    vector<double> target_joints2 = jupiter.getCurrentJointValues();
    target_joints1[0]=-1.35;
    target_joints1[1]=1.35;
    target_joints2[0]=-3.0;
    target_joints2[1]=3.0;
    while (not ros::isShuttingDown()){
        jupiter.setJointValueTarget(target_joints1);
        jupiter.move();
        sleep(0.5);
        jupiter.setJointValueTarget(target_joints2);
        jupiter.move();
        sleep(0.5);
    	// 设置随机目标点  Set random target points
    	jupiter.setRandomTarget();
    	// 开始移动  Start moving
    	jupiter.move();
    	sleep(0.5);
    }
    // 阻塞进程 Blocking process
    ros::waitForShutdown();
    return 0;
}

