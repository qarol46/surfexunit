#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <asump_localization/srv/get_waypoint.hpp>
#include <chrono>
#include <functional>
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
    MissionExecutor() : Node("mission_executor")
    {
        // ==================== Параметры ====================
        action_name_ = declare_parameter<std::string>("action_name", "navigate_to_pose");
        start_service_name_ = declare_parameter<std::string>("start_service_name", "/mission/start");
        waypoint_service_name_ = declare_parameter<std::string>("waypoint_service_name", "/mission/get_waypoint");
        estop_topic_ = declare_parameter<std::string>("estop_topic", "/estop");
        mission_status_topic_ = declare_parameter<std::string>("mission_status_topic", "/mission/status");
        retry_limit_ = declare_parameter<int>("retry_limit", 2);
        retry_delay_sec_ = declare_parameter<double>("retry_delay_sec", 2.0);
        skip_unreachable_ = declare_parameter<bool>("skip_unreachable", true);
        auto_start_ = declare_parameter<bool>("auto_start", false);  // ← ПО УМОЛЧАНИЮ FALSE!

        // ==================== Publishers / Subscribers ====================
        mission_status_pub_ = create_publisher<std_msgs::msg::String>(mission_status_topic_, 10);
        estop_sub_ = create_subscription<std_msgs::msg::Bool>(
            estop_topic_, 10, std::bind(&MissionExecutor::estopCallback, this, _1));

        // ==================== Сервис для ручного старта ====================
        start_mission_srv_ = create_service<Trigger>(
            "/start_coverage_mission",
            [this](const std::shared_ptr<Trigger::Request>,
                   std::shared_ptr<Trigger::Response> res) {
                RCLCPP_INFO(get_logger(), "Manual mission start requested");
                res->success = true;
                res->message = "Mission starting";
                state_ = State::IDLE;
                tryStartMission();
            });

        // ==================== Clients ====================
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, action_name_);
        start_client_ = create_client<Trigger>(start_service_name_);
        waypoint_client_ = create_client<GetWaypoint>(waypoint_service_name_);

        // ==================== Init timer ====================
        init_timer_ = create_wall_timer(1s, std::bind(&MissionExecutor::initTimerCallback, this));

        RCLCPP_INFO(get_logger(), "Mission executor initialized (auto_start=%s)",
                    auto_start_ ? "true" : "false");
    }

private:
    enum class State { IDLE, STARTING, EXECUTING, WAITING_RETRY, PAUSED_ESTOP, DONE, FAILED };

    void initTimerCallback()
    {
        if (!auto_start_ || estop_active_ || state_ != State::IDLE) return;
        tryStartMission();
    }

    void tryStartMission()
    {
        if (state_ != State::IDLE) return;

        if (!start_client_->wait_for_service(2s) || !waypoint_client_->wait_for_service(2s)) {
            publishMissionStatus("WAITING_FOR_MISSION_SERVICES");
            startRetryTimer(2.0);
            return;
        }

        state_ = State::STARTING;
        publishMissionStatus("REQUESTING_MISSION_START");

        auto request = std::make_shared<Trigger::Request>();
        start_client_->async_send_request(
            request,
            [this](rclcpp::Client<Trigger>::SharedFuture future) {
                try {
                    auto response = future.get();
                    if (!response->success) {
                        RCLCPP_WARN(get_logger(), "Mission start failed: %s", response->message.c_str());
                        state_ = State::IDLE;
                        publishMissionStatus("MISSION_START_FAILED");
                        return;
                    }
                    RCLCPP_INFO(get_logger(), "Mission started: %s", response->message.c_str());
                    publishMissionStatus("MISSION_STARTED");
                    requestWaypoint(true);
                } catch (const std::exception & e) {
                    RCLCPP_ERROR(get_logger(), "Exception in start callback: %s", e.what());
                    state_ = State::IDLE;
                    publishMissionStatus("START_EXCEPTION");
                }
            });
    }

    void requestWaypoint(bool advance)
    {
        if (estop_active_) return;
        if (!waypoint_client_->wait_for_service(2s)) {
            publishMissionStatus("WAITING_FOR_WAYPOINT_SERVICE");
            startRetryTimer(1.0);
            return;
        }

        auto request = std::make_shared<GetWaypoint::Request>();
        request->advance = advance;
        waypoint_client_->async_send_request(
            request,
            [this, advance](rclcpp::Client<GetWaypoint>::SharedFuture future) {
                try {
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
                    frame_id_ = response->pose.header.frame_id.empty() ? "map" : response->pose.header.frame_id;
                    retry_count_ = 0;
                    RCLCPP_INFO(get_logger(), "Waypoint %d/%d: (%.2f, %.2f)",
                                current_index_ + 1, total_waypoints_,
                                current_goal_pose_.position.x, current_goal_pose_.position.y);
                    publishMissionStatus("RECEIVED_WAYPOINT_" + std::to_string(current_index_ + 1));
                    sendCurrentGoal();
                } catch (const std::exception & e) {
                    RCLCPP_ERROR(get_logger(), "Exception in waypoint callback: %s", e.what());
                    handleFailure("WAYPOINT_EXCEPTION");
                }
            });
    }

    void sendCurrentGoal()
    {
        if (estop_active_) { state_ = State::PAUSED_ESTOP; return; }
        if (current_index_ < 0) { requestWaypoint(true); return; }

        if (!nav_client_->wait_for_action_server(5s)) {
            RCLCPP_WARN(get_logger(), "Action server %s not available", action_name_.c_str());
            publishMissionStatus("WAITING_FOR_ACTION_SERVER");
            startRetryTimer(2.0);
            return;
        }

        NavigateToPose::Goal goal_msg;
        goal_msg.pose.header.frame_id = frame_id_;
        goal_msg.pose.header.stamp = now();
        goal_msg.pose.pose = current_goal_pose_;

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            [this](const GoalHandleNav::SharedPtr & goal_handle) {
                current_goal_handle_ = goal_handle;
                if (!goal_handle) {
                    RCLCPP_ERROR(get_logger(), "Goal rejected by action server");
                    goal_active_ = false;
                    handleFailure("GOAL_REJECTED");
                }
            };
        send_goal_options.result_callback =
            [this](const GoalHandleNav::WrappedResult & result) {
                goal_active_ = false;
                current_goal_handle_ = nullptr;
                if (estop_active_ || state_ == State::PAUSED_ESTOP || state_ == State::WAITING_RETRY) return;

                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(get_logger(), "Waypoint %d/%d reached", current_index_ + 1, total_waypoints_);
                        publishMissionStatus("WAYPOINT_REACHED_" + std::to_string(current_index_ + 1));
                        requestWaypoint(true);
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        handleFailure("CANCELED");
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        handleFailure("ABORTED");
                        break;
                    default:
                        handleFailure("UNKNOWN");
                        break;
                }
            };

        state_ = State::EXECUTING;
        goal_active_ = true;
        nav_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void handleFailure(const std::string & reason)
    {
        if (estop_active_) return;
        if (retry_count_ < retry_limit_) {
            retry_count_++;
            RCLCPP_WARN(get_logger(), "Retry %d/%d. Reason: %s", retry_count_, retry_limit_, reason.c_str());
            state_ = State::WAITING_RETRY;
            startRetryTimer(retry_delay_sec_);
            return;
        }
        if (skip_unreachable_) {
            RCLCPP_WARN(get_logger(), "Skipping waypoint %d", current_index_ + 1);
            requestWaypoint(true);
            return;
        }
        state_ = State::FAILED;
        publishMissionStatus("FAILED_" + reason);
    }

    void missionCompleted()
    {
        state_ = State::DONE;
        RCLCPP_INFO(get_logger(), "=== MISSION COMPLETED ===");
        publishMissionStatus("MISSION_COMPLETED");
    }

    void estopCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        estop_active_ = msg->data;
        if (estop_active_) {
            state_ = State::PAUSED_ESTOP;
            if (current_goal_handle_) nav_client_->async_cancel_goal(current_goal_handle_);
        } else if (state_ == State::PAUSED_ESTOP) {
            if (current_index_ >= 0) sendCurrentGoal();
            else { state_ = State::IDLE; tryStartMission(); }
        }
    }

    void startRetryTimer(double delay_sec)
    {
        if (retry_timer_) retry_timer_->cancel();
        auto delay = std::chrono::milliseconds(static_cast<int>(delay_sec * 1000.0));
        retry_timer_ = create_wall_timer(delay, [this]() {
            retry_timer_.reset();
            if (state_ == State::WAITING_RETRY && !estop_active_) {
                if (current_index_ >= 0) sendCurrentGoal();
                else requestWaypoint(true);
            }
        });
    }

    void publishMissionStatus(const std::string & status)
    {
        std_msgs::msg::String msg;
        msg.data = status;
        mission_status_pub_->publish(msg);
        RCLCPP_INFO(get_logger(), "Status: %s", status.c_str());
    }

    // Members
    std::string action_name_, start_service_name_, waypoint_service_name_;
    std::string estop_topic_, mission_status_topic_;
    int retry_limit_;
    double retry_delay_sec_;
    bool skip_unreachable_, auto_start_;
    State state_ = State::IDLE;
    std::string frame_id_ = "map";
    geometry_msgs::msg::Pose current_goal_pose_;
    int current_index_ = -1, total_waypoints_ = 0, retry_count_ = 0;
    bool estop_active_ = false, goal_active_ = false;

    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    GoalHandleNav::SharedPtr current_goal_handle_;
    rclcpp::Client<Trigger>::SharedPtr start_client_;
    rclcpp::Client<GetWaypoint>::SharedPtr waypoint_client_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_status_pub_;
    rclcpp::Service<Trigger>::SharedPtr start_mission_srv_;
    rclcpp::TimerBase::SharedPtr init_timer_, retry_timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionExecutor>());
    rclcpp::shutdown();
    return 0;
}