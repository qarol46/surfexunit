#ifndef ASUMP_NAV__PATH_FOLLOWER_HPP_
#define ASUMP_NAV__PATH_FOLLOWER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Кастомный сервис планировщика
#include "asump_localization/srv/get_path_to_point.hpp"

#include <memory>
#include <string>
#include <cmath>

namespace autonomous_navigation
{

class PathFollower : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ServerGoalHandle<NavigateToPose>;
    using GetPathToPoint = asump_localization::srv::GetPathToPoint;

    explicit PathFollower(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~PathFollower() override = default;

private:
    // Action callbacks
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const NavigateToPose::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleNav> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleNav> goal_handle);
    void execute(const std::shared_ptr<GoalHandleNav> goal_handle);

    // Планирование пути через сервис
    bool plan_path(
        const geometry_msgs::msg::PoseStamped & goal,
        nav_msgs::msg::Path & out_path);

    // Control loop
    void follow_path(
        const std::shared_ptr<GoalHandleNav> goal_handle,
        const nav_msgs::msg::Path & path);

    // Pure pursuit helpers
    geometry_msgs::msg::PoseStamped get_current_pose();
    geometry_msgs::msg::PoseStamped find_lookahead_point(
        const geometry_msgs::msg::PoseStamped & current,
        const nav_msgs::msg::Path & path,
        double lookahead);
    double compute_linear_velocity(
        double dist_to_goal, double curvature, double current_vel);
    double compute_lookahead(double linear_vel);

    // Utils
    static double yaw_from_quat(const geometry_msgs::msg::Quaternion & q);
    static double normalize_angle(double a);
    static double distance_2d(
        const geometry_msgs::msg::Point & a,
        const geometry_msgs::msg::Point & b);

    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Publishers / clients
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Client<GetPathToPoint>::SharedPtr planner_client_;
    rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;

    // Parameters
    double max_linear_vel_;
    double max_angular_vel_;
    double min_approach_linear_vel_;
    double approach_velocity_scaling_dist_;
    double base_lookahead_dist_;
    double min_lookahead_dist_;
    double max_lookahead_dist_;
    double lookahead_time_;
    double goal_reached_tolerance_;
    double goal_yaw_tolerance_;
    double curvature_velocity_scaling_;
    double control_frequency_;
    double planner_timeout_;
    bool use_velocity_scaled_lookahead_;
    bool use_regulated_linear_velocity_scaling_;
    std::string robot_base_frame_;
    std::string global_frame_;
    std::string planner_service_name_;
};

}  // namespace autonomous_navigation

#endif  // AUTONOMOUS_NAVIGATION__PATH_FOLLOWER_HPP_