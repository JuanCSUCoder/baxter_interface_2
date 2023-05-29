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

    // TODO: MoveIt Baxter Rviz

    rclcpp::shutdown();
    return 0;
}