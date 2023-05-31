#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <baxter_interface_2/ik_interface.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "baxter_interface_test_node",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    auto const logger = rclcpp::get_logger("Baxter Interface Test Node");

    // Create the MoveIt MoveGroup Interface
    auto move_group_interface = moveit::planning_interface::MoveGroupInterface(node, "left_arm");

    // Get current pose
    auto const current_pose = move_group_interface.getCurrentPose().pose;

    RCLCPP_INFO(logger, "Orientation:");
    RCLCPP_INFO(logger, "W: %d", current_pose.orientation.w);
    RCLCPP_INFO(logger, "X: %d", current_pose.orientation.x);
    RCLCPP_INFO(logger, "Y: %d", current_pose.orientation.y);
    RCLCPP_INFO(logger, "Z: %d", current_pose.orientation.z);

    RCLCPP_INFO(logger, "Position:");
    RCLCPP_INFO(logger, "X: %d", current_pose.position.x);
    RCLCPP_INFO(logger, "Y: %d", current_pose.position.y);
    RCLCPP_INFO(logger, "Z: %d", current_pose.position.z);

    auto const target_pose = []
    {
        geometry_msgs::msg::Pose msg;
        
        msg.orientation.w = 1.0;
        msg.position.x = 0.5;
        msg.position.y = 0.5;
        msg.position.z = 0.5;

        return msg;
    }();

    move_group_interface.setPoseTarget(target_pose);

    // Create a plan to target pose
    auto const [success, plan] = [&move_group_interface]
    {
        moveit::planning_interface::MoveGroupInterface::Plan p;
        auto const ok = static_cast<bool>(move_group_interface.plan(p));
        return std::make_pair(ok, p);
    }();

    // Execute plan
    if (success)
    {
        move_group_interface.execute(plan);
    }
    else
    {
        RCLCPP_ERROR(logger, "Unable to plan movement");
    }

    rclcpp::shutdown();
    return 0;
}