#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <baxter_interface_2/ik_interface.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(0, nullptr);

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

    

    rclcpp::shutdown();
    return 0;
}