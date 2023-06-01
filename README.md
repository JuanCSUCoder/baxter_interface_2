# Baxter Interface for ROS2 Humble

![Lines of code](https://img.shields.io/tokei/lines/github/JuanCSUCoder/baxter_interface_2?style=for-the-badge)
![GitHub](https://img.shields.io/github/license/JuanCSUCoder/baxter_interface_2?style=for-the-badge)


Baxter Interface for ROS2 is a Software Development Kit (SDK) and Library for developing modern applications on Baxter Robotic Platform

## Example Code

```cpp
// Baxter Interface for ROS2 Library
#include <baxter_interface_2/robot.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto const target_pose = []
    {
        geometry_msgs::msg::Pose msg;
        
        // TODO: Insert Pose Values

        return msg;
    }();

    // Example Code
    BaxterRobot robot;

    robot.print_pose();
    robot.left_arm_to_target(target_pose);

    rclcpp::shutdown();
    return 0;
}
```

## Running on Moveit

TODO