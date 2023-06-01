#include <baxter_interface_2/robot.hpp>

BaxterRobot::BaxterRobot()
{
    this->node = std::make_shared<rclcpp::Node>(
        "baxter_interface_controller",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));


    auto const exec_pt = &this->executor;
    auto const node_pt = &this->node;
    this->exec_thread = std::make_unique<std::thread>([exec_pt, node_pt]()
                                                      { 
        exec_pt->add_node(*node_pt);
        exec_pt->spin(); 
        exec_pt->remove_node(*node_pt); });

    while (!this->executor.is_spinning())
    {
        RCLCPP_WARN(this->node->get_logger(), "Executor Not Spinning");
        using namespace std::chrono_literals;
        rclcpp::sleep_for(1s);
    }

    RCLCPP_INFO(this->node->get_logger(), "Executor Ready");
}

void BaxterRobot::print_the_pose(geometry_msgs::msg::Pose pose)
{
    RCLCPP_INFO(this->node->get_logger(), "Orientation:");
    RCLCPP_INFO(this->node->get_logger(), "W: %f", pose.orientation.w);
    RCLCPP_INFO(this->node->get_logger(), "X: %f", pose.orientation.x);
    RCLCPP_INFO(this->node->get_logger(), "Y: %f", pose.orientation.y);
    RCLCPP_INFO(this->node->get_logger(), "Z: %f", pose.orientation.z);

    RCLCPP_INFO(this->node->get_logger(), "Position:");
    RCLCPP_INFO(this->node->get_logger(), "X: %f", pose.position.x);
    RCLCPP_INFO(this->node->get_logger(), "Y: %f", pose.position.y);
    RCLCPP_INFO(this->node->get_logger(), "Z: %f", pose.position.z);
}

geometry_msgs::msg::Pose BaxterRobot::get_left_arm_pose()
{
    // Create the MoveIt MoveGroup Interface
    auto move_group_interface = moveit::planning_interface::MoveGroupInterface(node, "left_arm");

    // Get current pose
    return move_group_interface.getCurrentPose().pose;
}

geometry_msgs::msg::Pose BaxterRobot::get_right_arm_pose()
{
    // Create the MoveIt MoveGroup Interface
    auto move_group_interface = moveit::planning_interface::MoveGroupInterface(node, "right_arm");

    // Get current pose
    return move_group_interface.getCurrentPose().pose;
}

std::pair<geometry_msgs::msg::Pose, geometry_msgs::msg::Pose> BaxterRobot::get_pose()
{
    return std::make_pair<geometry_msgs::msg::Pose, geometry_msgs::msg::Pose>(this->get_left_arm_pose(), this->get_right_arm_pose());
}

void BaxterRobot::print_pose()
{
    RCLCPP_INFO(this->node->get_logger(), "LEFT ARM POSE ---------------------");
    this->print_the_pose(this->get_left_arm_pose());
    RCLCPP_INFO(this->node->get_logger(), "RIGHT ARM POSE ---------------------");
    this->print_the_pose(this->get_right_arm_pose());
}

bool BaxterRobot::left_arm_to_target(geometry_msgs::msg::Pose target_pose)
{
    auto move_group_interface = moveit::planning_interface::MoveGroupInterface(node, "left_arm");

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
        RCLCPP_ERROR(this->node->get_logger(), "Unable to plan movement");
    }

    return success;
}

bool BaxterRobot::right_arm_to_target(geometry_msgs::msg::Pose target_pose)
{
    auto move_group_interface = moveit::planning_interface::MoveGroupInterface(node, "right_arm");

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
        RCLCPP_ERROR(this->node->get_logger(), "Unable to plan movement");
    }

    return success;
}

BaxterRobot::~BaxterRobot()
{
}
