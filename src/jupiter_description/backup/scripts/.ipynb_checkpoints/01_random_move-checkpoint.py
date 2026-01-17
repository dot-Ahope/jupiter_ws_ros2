#!/usr/bin/env python
# coding: utf-8
from time import sleep
import rospy
from moveit_commander.move_group import MoveGroupCommander

if __name__ == '__main__':
    # 初始化节点 Initialize node
    rospy.init_node("jupiter_set_move")
    # 初始化机械臂 Initialize the robotic arm
    jupiter = MoveGroupCommander("arm")
    # 当运动规划失败后，允许重新规划 When motion planning fails, re-planning is allowed
    jupiter.allow_replanning(True)
    jupiter.set_planning_time(5)
    # 尝试规划的次数 Number of planning attempts
    jupiter.set_num_planning_attempts(10)
    # 设置允许目标位置误差 Set the allowable target position error
    jupiter.set_goal_position_tolerance(0.01)
    # 设置允许目标姿态误差 Set the allowable target attitude error
    jupiter.set_goal_orientation_tolerance(0.01)
    # 设置允许目标误差 Set the allowable target error
    jupiter.set_goal_tolerance(0.01)
    # 设置最大速度 Set maximum speed
    jupiter.set_max_velocity_scaling_factor(1.0)
    # 设置最大加速度 Set maximum acceleration
    jupiter.set_max_acceleration_scaling_factor(1.0)
    while not rospy.is_shutdown():
        # 设置随机目标点 Set random target points
        jupiter.set_random_target()
        # 开始运动 Start 
        jupiter.go()
        sleep(0.5)
        # 设置"pose1"为目标点  Set "pose1" as the target point
        #jupiter.set_named_target("pose1")
        #jupiter.go()
        #sleep(0.5)
        # 设置"pose2"为目标点  Set "pose2" as the target point
        #jupiter.set_named_target("pose2")
        #jupiter.go()
        #sleep(0.5)
