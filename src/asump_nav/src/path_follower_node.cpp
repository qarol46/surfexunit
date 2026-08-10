#include <rclcpp/rclcpp.hpp>
#include "asump_nav/path_follower.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<autonomous_navigation::PathFollower>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}