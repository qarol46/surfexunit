#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <std_srvs/srv/trigger.hpp>
#include <asump_localization/srv/get_waypoint.hpp>

#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

using namespace std::placeholders;
using namespace std::chrono_literals;

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

using Trigger = std_srvs::srv::Trigger;
using GetWaypoint = asump_localization::srv::GetWaypoint;

class MissionExecutor : public rclcpp::Node
{
public:
  MissionExecutor()
  : Node("mission_executor")
  {
    // ==================== Параметры ====================
    action_name_ = declare_parameter<std::string>("action_name", "navigate_to_pose");

    start_service_name_ = declare_parameter<std::string>(
      "start_service_name", "/mission/start");

    waypoint_service_name_ = declare_parameter<std::string>(
      "waypoint_service_name", "/mission/get_waypoint");

    estop_topic_ = declare_parameter<std::string>(
      "estop_topic", "/estop");

    obstacle_replan_topic_ = declare_parameter<std::string>(
      "obstacle_replan_topic", "/obstacle/replan");

    nav_status_topic_ = declare_parameter<std::string>(
      "nav_status_topic", "/path_follower/nav_status");

    mission_status_topic_ = declare_parameter<std::string>(
      "mission_status_topic", "/mission/status");

    retry_limit_ = declare_parameter<int>("retry_limit", 2);
    retry_delay_sec_ = declare_parameter<double>("retry_delay_sec", 2.0);
    skip_unreachable_ = declare_parameter<bool>("skip_unreachable", true);
    auto_start_ = declare_parameter<bool>("auto_start", true);

    // ==================== Publishers / Subscribers ====================
    mission_status_pub_ = create_publisher<std_msgs::msg::String>(
      mission_status_topic_, 10);

    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
      estop_topic_,
      10,
      std::bind(&MissionExecutor::estopCallback, this, _1));

    obstacle_replan_sub_ = create_subscription<std_msgs::msg::Bool>(
      obstacle_replan_topic_,
      10,
      std::bind(&MissionExecutor::obstacleReplanCallback, this, _1));

    rclcpp::QoS nav_status_qos(10);
    nav_status_qos.transient_local();

    nav_status_sub_ = create_subscription<std_msgs::msg::String>(
      nav_status_topic_,
      nav_status_qos,
      std::bind(&MissionExecutor::navStatusCallback, this, _1));

    // ==================== Clients ====================
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, action_name_);

    start_client_ = create_client<Trigger>(start_service_name_);
    waypoint_client_ = create_client<GetWaypoint>(waypoint_service_name_);

    // ==================== Init timer ====================
    init_timer_ = create_wall_timer(
      1s,
      std::bind(&MissionExecutor::initTimerCallback, this));

    RCLCPP_INFO(get_logger(), "Mission executor initialized.");
    RCLCPP_INFO(get_logger(), "Action name: %s", action_name_.c_str());
    RCLCPP_INFO(get_logger(), "Start service: %s", start_service_name_.c_str());
    RCLCPP_INFO(get_logger(), "Waypoint service: %s", waypoint_service_name_.c_str());
    RCLCPP_INFO(get_logger(), "Auto start: %s", auto_start_ ? "true" : "false");
  }

private:
  enum class State
  {
    IDLE,
    STARTING,
    EXECUTING,
    WAITING_RETRY,
    PAUSED_ESTOP,
    DONE,
    FAILED
  };

  // ==================== Init ====================

  void initTimerCallback()
  {
    if (!auto_start_) {
      return;
    }

    if (estop_active_) {
      return;
    }

    if (state_ == State::IDLE) {
      tryStartMission();
    }
  }

  void tryStartMission()
  {
    if (state_ != State::IDLE) {
      return;
    }

    if (!start_client_->wait_for_service(0s) ||
        !waypoint_client_->wait_for_service(0s))
    {
      publishMissionStatus("WAITING_FOR_MISSION_SERVICES");
      return;
    }

    state_ = State::STARTING;
    publishMissionStatus("REQUESTING_MISSION_START");

    auto request = std::make_shared<Trigger::Request>();

    start_client_->async_send_request(
      request,
      std::bind(&MissionExecutor::startResponseCallback, this, _1));
  }

  void startResponseCallback(
    rclcpp::Client<Trigger>::SharedFuture future)
  {
    auto response = future.get();

    if (!response->success) {
      RCLCPP_WARN(
        get_logger(),
        "Mission start failed: %s",
        response->message.c_str());

      state_ = State::IDLE;
      publishMissionStatus("MISSION_START_FAILED");
      return;
    }

    RCLCPP_INFO(get_logger(), "Mission started: %s", response->message.c_str());
    publishMissionStatus("MISSION_STARTED");

    requestWaypoint(true);
  }

  // ==================== Waypoint service ====================

  void requestWaypoint(bool advance)
  {
    if (estop_active_) {
      return;
    }

    if (!waypoint_client_->wait_for_service(0s)) {
      publishMissionStatus("WAITING_FOR_WAYPOINT_SERVICE");
      state_ = State::WAITING_RETRY;
      startRetryTimer(1.0);
      return;
    }

    auto request = std::make_shared<GetWaypoint::Request>();
    request->advance = advance;

    waypoint_client_->async_send_request(
      request,
      [this, advance](rclcpp::Client<GetWaypoint>::SharedFuture future) {
        this->waypointResponseCallback(future, advance);
      });
  }

  void waypointResponseCallback(
    rclcpp::Client<GetWaypoint>::SharedFuture future,
    bool /*advance*/)
  {
    auto response = future.get();

    if (!response->success) {
      if (response->finished) {
        missionCompleted();
      } else {
        handleFailure("WAYPOINT_SERVICE_ERROR");
      }

      return;
    }

    current_index_ = response->index;
    total_waypoints_ = response->total;
    current_goal_pose_ = response->pose.pose;

    frame_id_ = response->pose.header.frame_id.empty()
      ? "map"
      : response->pose.header.frame_id;

    retry_count_ = 0;

    RCLCPP_INFO(
      get_logger(),
      "Received waypoint %d/%d: (%.2f, %.2f)",
      current_index_ + 1,
      total_waypoints_,
      current_goal_pose_.position.x,
      current_goal_pose_.position.y);

    publishMissionStatus(
      "RECEIVED_WAYPOINT_" + std::to_string(current_index_ + 1));

    sendCurrentGoal();
  }

  // ==================== Action logic ====================

  void sendCurrentGoal()
  {
    if (estop_active_) {
      state_ = State::PAUSED_ESTOP;
      return;
    }

    if (current_index_ < 0) {
      requestWaypoint(true);
      return;
    }

    if (!nav_client_->wait_for_action_server(1s)) {
      RCLCPP_WARN(
        get_logger(),
        "Action server %s not available. Retrying...",
        action_name_.c_str());

      publishMissionStatus("WAITING_FOR_ACTION_SERVER");

      state_ = State::WAITING_RETRY;
      startRetryTimer(1.0);
      return;
    }

    NavigateToPose::Goal goal_msg;
    goal_msg.pose.header.frame_id = frame_id_;
    goal_msg.pose.header.stamp = now();
    goal_msg.pose.pose = current_goal_pose_;

    RCLCPP_INFO(
      get_logger(),
      "Sending waypoint %d/%d to action server: (%.2f, %.2f)",
      current_index_ + 1,
      total_waypoints_,
      goal_msg.pose.pose.position.x,
      goal_msg.pose.pose.position.y);

    publishMissionStatus(
      "NAVIGATING_TO_WAYPOINT_" + std::to_string(current_index_ + 1));

    auto send_goal_options =
      rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    send_goal_options.goal_response_callback =
      std::bind(&MissionExecutor::goalResponseCallback, this, _1);

    send_goal_options.result_callback =
      std::bind(&MissionExecutor::resultCallback, this, _1);

    send_goal_options.feedback_callback =
      std::bind(&MissionExecutor::feedbackCallback, this, _1, _2);

    state_ = State::EXECUTING;
    goal_active_ = true;

    nav_client_->async_send_goal(goal_msg, send_goal_options);
  }

  void goalResponseCallback(const GoalHandleNav::SharedPtr & goal_handle)
  {
    current_goal_handle_ = goal_handle;

    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "Goal was rejected by action server.");
      goal_active_ = false;
      handleFailure("GOAL_REJECTED");
    }
  }

  void feedbackCallback(
    GoalHandleNav::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  {
    last_distance_remaining_ = feedback->distance_remaining;

    RCLCPP_DEBUG(
      get_logger(),
      "Feedback: distance_remaining = %.3f",
      feedback->distance_remaining);
  }

  void resultCallback(const GoalHandleNav::WrappedResult & result)
  {
    goal_active_ = false;
    current_goal_handle_ = nullptr;

    if (estop_active_ || state_ == State::PAUSED_ESTOP) {
      return;
    }

    if (state_ == State::WAITING_RETRY) {
      return;
    }

    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(
          get_logger(),
          "Waypoint %d/%d reached.",
          current_index_ + 1,
          total_waypoints_);

        publishMissionStatus(
          "WAYPOINT_REACHED_" + std::to_string(current_index_ + 1));

        requestWaypoint(true);
        break;

      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(get_logger(), "Navigation canceled.");
        handleFailure("CANCELED");
        break;

      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_WARN(
          get_logger(),
          "Navigation aborted. Last path_follower status: %s",
          last_nav_status_.c_str());

        handleFailure(last_nav_status_);
        break;

      default:
        RCLCPP_ERROR(get_logger(), "Unknown action result code.");
        handleFailure("UNKNOWN");
        break;
    }
  }

  // ==================== Failure handling ====================

  void handleFailure(const std::string & reason)
  {
    if (estop_active_) {
      return;
    }

    if (reason == "CANCELED") {
      state_ = State::FAILED;
      publishMissionStatus("FAILED_CANCELED");
      return;
    }

    bool retryable =
      reason == "PATH_NOT_FOUND" ||
      reason == "EMPTY_PATH" ||
      reason == "PLANNER_ERROR" ||
      reason == "PLANNER_TIMEOUT" ||
      reason == "PLANNER_SERVICE_UNAVAILABLE" ||
      reason == "GOAL_REJECTED" ||
      reason == "STUCK" ||
      reason == "UNKNOWN" ||
      reason == "WAYPOINT_SERVICE_ERROR";

    if (!retryable) {
      state_ = State::FAILED;
      publishMissionStatus("FAILED_" + reason);
      return;
    }

    if (retry_count_ < retry_limit_) {
      retry_count_++;

      RCLCPP_WARN(
        get_logger(),
        "Retry %d/%d for waypoint %d. Reason: %s",
        retry_count_,
        retry_limit_,
        current_index_ + 1,
        reason.c_str());

      publishMissionStatus(
        "RETRY_WAYPOINT_" + std::to_string(current_index_ + 1) +
        "_REASON_" + reason);

      state_ = State::WAITING_RETRY;
      startRetryTimer(retry_delay_sec_);
      return;
    }

    if (skip_unreachable_) {
      RCLCPP_WARN(
        get_logger(),
        "Waypoint %d unreachable after retries. Skipping.",
        current_index_ + 1);

      publishMissionStatus(
        "SKIPPING_WAYPOINT_" + std::to_string(current_index_ + 1));

      requestWaypoint(true);
      return;
    }

    state_ = State::FAILED;
    publishMissionStatus("FAILED_AFTER_RETRIES_" + reason);
  }

  void missionCompleted()
  {
    state_ = State::DONE;

    RCLCPP_INFO(get_logger(), "=== MISSION COMPLETED ===");
    publishMissionStatus("MISSION_COMPLETED");
  }

  // ==================== E-Stop / obstacle ====================

  void estopCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    bool new_estop = msg->data;

    if (new_estop && !estop_active_) {
      estop_active_ = true;
      state_ = State::PAUSED_ESTOP;

      RCLCPP_WARN(get_logger(), "E-STOP active. Pausing mission.");
      publishMissionStatus("ESTOP_PAUSED");

      cancelActiveGoal();
      return;
    }

    if (!new_estop && estop_active_) {
      estop_active_ = false;

      if (state_ == State::PAUSED_ESTOP) {
        RCLCPP_INFO(get_logger(), "E-STOP released. Resuming mission.");
        publishMissionStatus("ESTOP_RESUMED");

        if (current_index_ >= 0) {
          sendCurrentGoal();
        } else {
          state_ = State::IDLE;
          tryStartMission();
        }
      }
    }
  }

  void obstacleReplanCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data) {
      return;
    }

    if (estop_active_) {
      return;
    }

    if (state_ != State::EXECUTING) {
      return;
    }

    RCLCPP_WARN(get_logger(), "Obstacle replan requested.");
    publishMissionStatus("OBSTACLE_REPLAN");

    state_ = State::WAITING_RETRY;
    cancelActiveGoal();
    startRetryTimer(0.5);
  }

  void navStatusCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    last_nav_status_ = msg->data;
  }

  // ==================== Helpers ====================

  void cancelActiveGoal()
  {
    if (!nav_client_) {
      return;
    }

    if (current_goal_handle_) {
      RCLCPP_INFO(get_logger(), "Canceling current navigation goal.");
      nav_client_->async_cancel_goal(current_goal_handle_);
      return;
    }

    if (goal_active_) {
      RCLCPP_INFO(get_logger(), "Canceling all navigation goals.");
      nav_client_->async_cancel_all_goals();
    }
  }

  void startRetryTimer(double delay_sec)
  {
    if (retry_timer_) {
      retry_timer_->cancel();
      retry_timer_.reset();
    }

    if (delay_sec <= 0.0) {
      if (state_ == State::WAITING_RETRY && !estop_active_) {
        if (current_index_ >= 0) {
          sendCurrentGoal();
        } else {
          requestWaypoint(true);
        }
      }

      return;
    }

    auto delay = std::chrono::milliseconds(
      static_cast<int>(delay_sec * 1000.0));

    retry_timer_ = create_wall_timer(
      delay,
      [this]() {
        if (retry_timer_) {
          retry_timer_->cancel();
          retry_timer_.reset();
        }

        if (state_ == State::WAITING_RETRY && !estop_active_) {
          if (current_index_ >= 0) {
            sendCurrentGoal();
          } else {
            requestWaypoint(true);
          }
        }
      });
  }

  void publishMissionStatus(const std::string & status)
  {
    std_msgs::msg::String msg;
    msg.data = status;
    mission_status_pub_->publish(msg);

    RCLCPP_INFO(get_logger(), "Mission status: %s", status.c_str());
  }

  // ==================== Members ====================

  std::string action_name_;
  std::string start_service_name_;
  std::string waypoint_service_name_;

  std::string estop_topic_;
  std::string obstacle_replan_topic_;
  std::string nav_status_topic_;
  std::string mission_status_topic_;

  int retry_limit_;
  double retry_delay_sec_;
  bool skip_unreachable_;
  bool auto_start_;

  State state_ = State::IDLE;

  std::string frame_id_ = "map";
  geometry_msgs::msg::Pose current_goal_pose_;

  int current_index_ = -1;
  int total_waypoints_ = 0;
  int retry_count_ = 0;

  bool estop_active_ = false;
  bool goal_active_ = false;

  std::string last_nav_status_ = "UNKNOWN";
  float last_distance_remaining_ = 0.0f;

  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  GoalHandleNav::SharedPtr current_goal_handle_;

  rclcpp::Client<Trigger>::SharedPtr start_client_;
  rclcpp::Client<GetWaypoint>::SharedPtr waypoint_client_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr obstacle_replan_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr nav_status_sub_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_status_pub_;

  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr retry_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionExecutor>());
  rclcpp::shutdown();
  return 0;
}