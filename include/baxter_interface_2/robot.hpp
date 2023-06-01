#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

class BaxterRobot
{
private:
    rclcpp::Node::SharedPtr node;
    rclcpp::executors::MultiThreadedExecutor executor;

    void print_the_pose(geometry_msgs::msg::Pose pose);

public:
    BaxterRobot();
    ~BaxterRobot();

    geometry_msgs::msg::Pose get_left_arm_pose();
    geometry_msgs::msg::Pose get_right_arm_pose();
    std::pair<geometry_msgs::msg::Pose, geometry_msgs::msg::Pose> get_pose();

    void print_pose();

    bool left_arm_to_target(geometry_msgs::msg::Pose target_pose);
    bool right_arm_to_target(geometry_msgs::msg::Pose target_pose);
};