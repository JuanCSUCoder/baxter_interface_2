#include <baxter_interface_2/robot.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto const target_pose = []
    {
        geometry_msgs::msg::Pose msg;
        
        msg.orientation.w = 0.365370;
        msg.orientation.x = 0.804754;
        msg.orientation.y = 0.329520;
        msg.orientation.z = 0.332104;

        msg.position.x = 0.985046;
        msg.position.y = -0.088315;
        msg.position.z = 0.231450;

        return msg;
    }();

    auto const logger = rclcpp::get_logger("Baxter Tests");

    RCLCPP_INFO(logger, "Started test!");

    BaxterRobot robot;

    robot.print_pose();
    robot.left_arm_to_target(target_pose);

    RCLCPP_INFO(logger, "Terminating ...");

    rclcpp::shutdown();
    return 0;
}