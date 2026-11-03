// Нода докования на зарядную станцию.
//
// Сервис std_srvs/Trigger "dock" запускает двухфазный заезд:
//  1) подход: NavigateToPose (path_follower) в точку перед пандусом,
//     робот встаёт задом к станции;
//  2) серво-задание: реверс с выравниванием по ArUco-маркеру в картинке
//     задней камеры, финальная остановка у башни по задним УЗ-датчикам
//     (маркер вблизи становится слишком крупным и выходит из кадра).

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

namespace autonomous_navigation
{

using namespace std::chrono_literals;

class DockController : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;

  explicit DockController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("dock_controller", options)
  {
    // Топики и имена
    service_name_ = declare_parameter<std::string>("service_name", "dock");
    action_name_ = declare_parameter<std::string>("action_name", "navigate_to_pose");
    charger_pose_topic_ =
      declare_parameter<std::string>("charger_pose_topic", "/zones/charger_pose");
    image_topic_ = declare_parameter<std::string>("image_topic", "/back_camera/image");
    camera_info_topic_ =
      declare_parameter<std::string>("camera_info_topic", "/back_camera/camera_info");
    us_left_topic_ = declare_parameter<std::string>("us_left_topic", "/us/back_left");
    us_right_topic_ = declare_parameter<std::string>("us_right_topic", "/us/back_right");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    debug_image_topic_ =
      declare_parameter<std::string>("debug_image_topic", "/dock/debug_image");
    standoff_goal_topic_ =
      declare_parameter<std::string>("standoff_goal_topic", "/dock/standoff_goal");

    // Маркер
    marker_id_ = declare_parameter<int>("marker_id", 0);
    marker_size_ = declare_parameter<double>("marker_size", 0.2);
    fallback_fx_ = declare_parameter<double>("fallback_fx", 466.0);

    // Геометрия станции и точка предварительного подъезда
    ramp_offset_ = declare_parameter<double>("ramp_offset", 1.084);
    standoff_dist_ = declare_parameter<double>("standoff_dist", 0.8);

    // Таймауты подхода
    charger_wait_timeout_ = declare_parameter<double>("charger_wait_timeout", 5.0);
    nav_server_wait_timeout_ = declare_parameter<double>("nav_server_wait_timeout", 10.0);
    nav_result_timeout_ = declare_parameter<double>("nav_result_timeout", 120.0);

    // Серво-фаза
    servo_linear_vel_ = declare_parameter<double>("servo_linear_vel", 0.1);
    servo_angular_gain_ = declare_parameter<double>("servo_angular_gain", 1.2);
    servo_rate_ = declare_parameter<double>("servo_rate", 20.0);
    servo_timeout_ = declare_parameter<double>("servo_timeout", 60.0);
    marker_acquire_timeout_ = declare_parameter<double>("marker_acquire_timeout", 10.0);
    marker_lost_timeout_ = declare_parameter<double>("marker_lost_timeout", 2.0);
    marker_handoff_dist_ = declare_parameter<double>("marker_handoff_dist", 0.7);
    us_stop_distance_ = declare_parameter<double>("us_stop_distance", 0.3);
    us_emergency_distance_ = declare_parameter<double>("us_emergency_distance", 0.15);
    us_wait_timeout_ = declare_parameter<double>("us_wait_timeout", 3.0);
    publish_debug_image_ = declare_parameter<bool>("publish_debug_image", true);

    // Поиск маркера покачиванием на месте в начале серво-фазы
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    marker_search_enabled_ = declare_parameter<bool>("marker_search_enabled", true);
    marker_search_angle_ = declare_parameter<double>("marker_search_angle", 0.52);
    marker_search_angular_vel_ =
      declare_parameter<double>("marker_search_angular_vel", 0.3);

    cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    standoff_goal_pub_ =
      create_publisher<geometry_msgs::msg::PoseStamped>(standoff_goal_topic_, 10);
    if (publish_debug_image_) {
      debug_img_pub_ = create_publisher<sensor_msgs::msg::Image>(debug_image_topic_, 1);
    }

    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, action_name_, cb_group_);

    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = cb_group_;

    charger_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      charger_pose_topic_, rclcpp::QoS(10),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mutex_);
        charger_pose_ = *msg;
        have_charger_pose_ = true;
      },
      sub_opts);

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mutex_);
        last_image_ = msg;
      },
      sub_opts);

    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, rclcpp::QoS(10),
      [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        if (msg->k[0] > 0.0) {
          std::lock_guard<std::mutex> lk(mutex_);
          fx_ = msg->k[0];
        }
      },
      sub_opts);

    us_left_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      us_left_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) { on_us(msg, 0); },
      sub_opts);
    us_right_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      us_right_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) { on_us(msg, 1); },
      sub_opts);

    dock_srv_ = create_service<std_srvs::srv::Trigger>(
      service_name_,
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        on_dock_request(response);
      },
      rclcpp::ServicesQoS(), cb_group_);

    RCLCPP_INFO(get_logger(), "Dock controller ready: service /%s", service_name_.c_str());
  }

private:
  // Минимальная валидная дальность одного УЗ-датчика
  void on_us(const sensor_msgs::msg::LaserScan::SharedPtr & msg, size_t idx)
  {
    float best = std::numeric_limits<float>::infinity();
    for (const float r : msg->ranges) {
      if (std::isfinite(r) && r >= msg->range_min && r <= msg->range_max) {
        best = std::min(best, r);
      }
    }
    std::lock_guard<std::mutex> lk(mutex_);
    us_range_[idx] = best;
    us_stamp_[idx] = now();
  }

  double min_us_range() const
  {
    std::lock_guard<std::mutex> lk(mutex_);
    return static_cast<double>(std::min(us_range_[0], us_range_[1]));
  }

  static double yaw_from_quat(const geometry_msgs::msg::Quaternion & q)
  {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  static double normalize_angle(double a)
  {
    while (a > M_PI) {a -= 2.0 * M_PI;}
    while (a < -M_PI) {a += 2.0 * M_PI;}
    return a;
  }

  // Текущий yaw робота в фиксированной рамке (для отслеживания покачивания)
  bool current_yaw(const std::string & fixed_frame, double & yaw)
  {
    try {
      const auto tf = tf_buffer_->lookupTransform(fixed_frame, base_frame_, tf2::TimePointZero);
      yaw = yaw_from_quat(tf.transform.rotation);
      return true;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF lookup: %s", e.what());
      return false;
    }
  }

  void stop_robot()
  {
    geometry_msgs::msg::Twist stop;
    cmd_vel_pub_->publish(stop);
  }

  void on_dock_request(const std::shared_ptr<std_srvs::srv::Trigger::Response> & response)
  {
    bool expected = false;
    if (!active_.compare_exchange_strong(expected, true)) {
      response->success = false;
      response->message = "docking already in progress";
      return;
    }
    std::string message;
    response->success = execute_docking(message);
    response->message = message;
    stop_robot();
    active_ = false;
  }

  bool execute_docking(std::string & message)
  {
    // --- Фаза 0: поза зарядной станции (map frame) ---
    geometry_msgs::msg::PoseStamped charger;
    {
      const auto deadline = now() + rclcpp::Duration::from_seconds(charger_wait_timeout_);
      while (rclcpp::ok()) {
        {
          std::lock_guard<std::mutex> lk(mutex_);
          if (have_charger_pose_) {
            charger = charger_pose_;
            break;
          }
        }
        if (now() > deadline) {
          message = "no charger pose on " + charger_pose_topic_;
          return false;
        }
        rclcpp::sleep_for(50ms);
      }
    }

    // Точка подъезда: за пандусом, задом к станции (робот смотрит ОТ станции).
    // yaw позы зарядки — направление от башни к пандусу.
    const double yaw = yaw_from_quat(charger.pose.orientation);
    // path_planner и path_follower считают единичный кватернион (yaw ≈ 0)
    // признаком «ориентация не задана» и пропускают финальный доворот.
    // Поэтому почти нулевой yaw заменяем малым ненулевым (0.01 рад ≈ 0.6°,
    // в пределах goal_yaw_tolerance), чтобы доворот задом к станции выполнялся.
    double goal_yaw = yaw;
    if (std::abs(goal_yaw) < 1e-3) {
      goal_yaw = 0.01;
    }
    const double off = ramp_offset_ + standoff_dist_;
    geometry_msgs::msg::PoseStamped standoff;
    standoff.header.frame_id = charger.header.frame_id;
    standoff.header.stamp = now();
    standoff.pose.position.x = charger.pose.position.x + off * std::cos(yaw);
    standoff.pose.position.y = charger.pose.position.y + off * std::sin(yaw);
    standoff.pose.position.z = 0.0;
    standoff.pose.orientation.x = 0.0;
    standoff.pose.orientation.y = 0.0;
    standoff.pose.orientation.z = std::sin(goal_yaw / 2.0);
    standoff.pose.orientation.w = std::cos(goal_yaw / 2.0);

    standoff_goal_pub_->publish(standoff);

    RCLCPP_INFO(
      get_logger(), "Docking: approach to standoff (%.2f, %.2f), yaw %.2f",
      standoff.pose.position.x, standoff.pose.position.y, goal_yaw);

    // --- Фаза 1: подход через path_follower (action navigate_to_pose) ---
    if (!nav_client_->wait_for_action_server(
          std::chrono::duration<double>(nav_server_wait_timeout_))) {
      message = "navigate_to_pose action server not available";
      return false;
    }

    NavigateToPose::Goal goal;
    goal.pose = standoff;
    auto goal_handle_future = nav_client_->async_send_goal(goal);
    if (goal_handle_future.wait_for(std::chrono::duration<double>(nav_server_wait_timeout_)) !=
        std::future_status::ready) {
      message = "send goal timeout";
      return false;
    }
    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      message = "approach goal rejected";
      return false;
    }
    auto result_future = nav_client_->async_get_result(goal_handle);
    if (result_future.wait_for(std::chrono::duration<double>(nav_result_timeout_)) !=
        std::future_status::ready) {
      nav_client_->async_cancel_goal(goal_handle);
      message = "approach timeout";
      return false;
    }
    auto result = result_future.get();
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      message = "approach failed (action aborted/canceled)";
      return false;
    }

    RCLCPP_INFO(get_logger(), "Docking: standoff reached, starting visual servoing");

    // --- Фаза 2: серво-заезд реверсом по маркеру, финиш по УЗ ---
    return servo_reverse(message, charger.header.frame_id);
  }

  bool servo_reverse(std::string & message, const std::string & fixed_frame)
  {
    enum class Phase { MARKER, FINAL };
    Phase phase = Phase::MARKER;

    const auto start = now();
    const auto marker_wait_start = now();
    auto last_seen = now();
    bool ever_seen = false;
    auto final_phase_start = start;

    // Состояние поискового покачивания (пока маркер ни разу не найден)
    bool search_initialized = false;
    double search_start_yaw = 0.0;
    double search_dir = 1.0;

    rclcpp::Rate rate(servo_rate_);
    while (rclcpp::ok()) {
      if (now() - start > rclcpp::Duration::from_seconds(servo_timeout_)) {
        message = "servoing timeout";
        return false;
      }

      // Аварийная близость препятствия сзади
      const double us = min_us_range();
      if (us < us_emergency_distance_) {
        message = "emergency stop: rear obstacle too close";
        return false;
      }

      // Детекция маркера на последнем кадре
      sensor_msgs::msg::Image::SharedPtr img_msg;
      double fx = fallback_fx_;
      {
        std::lock_guard<std::mutex> lk(mutex_);
        img_msg = last_image_;
        fx = fx_;
      }

      bool detected = false;
      double err = 0.0;
      double marker_dist = std::numeric_limits<double>::infinity();

      if (img_msg) {
        cv::Mat gray;
        try {
          cv::Mat img = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::RGB8)->image;
          cv::cvtColor(img, gray, cv::COLOR_RGB2GRAY);
        } catch (const cv_bridge::Exception & e) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "cv_bridge: %s", e.what());
          rate.sleep();
          continue;
        }

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners, rejected;
        cv::aruco::detectMarkers(gray, aruco_dict_, corners, ids, aruco_params_);

        size_t idx = ids.size();
        for (size_t i = 0; i < ids.size(); ++i) {
          if (ids[i] == marker_id_) {
            idx = i;
            break;
          }
        }

        if (idx < ids.size()) {
          const auto & c = corners[idx];
          double cx = 0.0, width_sum = 0.0;
          for (int k = 0; k < 4; ++k) {
            cx += c[k].x;
            width_sum += cv::norm(c[k] - c[(k + 1) % 4]);
          }
          cx /= 4.0;
          const double px_width = width_sum / 4.0;
          err = (cx - gray.cols / 2.0) / (gray.cols / 2.0);
          marker_dist = fx * marker_size_ / px_width;
          detected = true;
        }

        if (publish_debug_image_ && debug_img_pub_) {
          cv::Mat dbg;
          cv::cvtColor(gray, dbg, cv::COLOR_GRAY2RGB);
          if (!ids.empty()) {
            cv::aruco::drawDetectedMarkers(dbg, corners, ids);
          }
          auto out = cv_bridge::CvImage(
            img_msg->header, sensor_msgs::image_encodings::RGB8, dbg);
          debug_img_pub_->publish(*out.toImageMsg());
        }
      }

      geometry_msgs::msg::Twist cmd;

      if (phase == Phase::MARKER) {
        if (detected) {
          ever_seen = true;
          last_seen = now();
          if (marker_dist <= marker_handoff_dist_) {
            phase = Phase::FINAL;
            final_phase_start = now();
            RCLCPP_INFO(
              get_logger(), "Docking: marker at %.2f m, switching to final approach",
              marker_dist);
            rate.sleep();
            continue;
          }
          // Маркер левее центра кадра (err < 0) -> нужен поворот по часовой
          // (задняя камера смотрит назад), поэтому знак прямой.
          cmd.linear.x = -servo_linear_vel_;
          cmd.angular.z = servo_angular_gain_ * err;
        } else {
          if (!ever_seen) {
            if (now() - marker_wait_start >
                rclcpp::Duration::from_seconds(marker_acquire_timeout_)) {
              message = "aruco marker not visible from standoff";
              return false;
            }
            // Маркера ещё не видно — поисковое покачивание на месте
            // ±marker_search_angle, чтобы захватить маркер задней камерой.
            // При недоступном TF — прежний медленный реверс "вслепую".
            double cy = 0.0;
            if (marker_search_enabled_ && current_yaw(fixed_frame, cy)) {
              if (!search_initialized) {
                search_start_yaw = cy;
                search_initialized = true;
                RCLCPP_INFO(
                  get_logger(),
                  "Docking: marker not visible, sweeping +/-%.0f deg to find it",
                  marker_search_angle_ * 180.0 / M_PI);
              }
              const double delta = normalize_angle(cy - search_start_yaw);
              if (delta >= marker_search_angle_) {
                search_dir = -1.0;
              } else if (delta <= -marker_search_angle_) {
                search_dir = 1.0;
              }
              cmd.angular.z = search_dir * marker_search_angular_vel_;
            } else {
              cmd.linear.x = -servo_linear_vel_;
            }
          } else if (now() - last_seen >
                     rclcpp::Duration::from_seconds(marker_lost_timeout_)) {
            message = "aruco marker lost during approach";
            return false;
          }
        }
      } else {  // Phase::FINAL
        // Финиш: прямой реверс до срабатывания УЗ-датчиков о башню
        if (std::isinf(us)) {
          if (now() - final_phase_start >
              rclcpp::Duration::from_seconds(us_wait_timeout_)) {
            message = "no rear ultrasonic data in final approach";
            return false;
          }
        } else if (us <= us_stop_distance_) {
          stop_robot();
          message = "docked successfully (rear us distance " +
                    std::to_string(us).substr(0, 4) + " m)";
          RCLCPP_INFO(get_logger(), "Docking: %s", message.c_str());
          return true;
        }
        cmd.linear.x = -servo_linear_vel_;
        cmd.angular.z = 0.0;
      }

      cmd_vel_pub_->publish(cmd);
      rate.sleep();
    }

    message = "ros shutdown during docking";
    return false;
  }

  // --- Параметры ---
  std::string service_name_, action_name_;
  std::string charger_pose_topic_, image_topic_, camera_info_topic_;
  std::string us_left_topic_, us_right_topic_, cmd_vel_topic_, debug_image_topic_;
  std::string standoff_goal_topic_;
  int marker_id_;
  double marker_size_, fallback_fx_;
  double ramp_offset_, standoff_dist_;
  double charger_wait_timeout_, nav_server_wait_timeout_, nav_result_timeout_;
  double servo_linear_vel_, servo_angular_gain_, servo_rate_, servo_timeout_;
  double marker_acquire_timeout_, marker_lost_timeout_, marker_handoff_dist_;
  double us_stop_distance_, us_emergency_distance_, us_wait_timeout_;
  bool publish_debug_image_;
  std::string base_frame_;
  bool marker_search_enabled_;
  double marker_search_angle_, marker_search_angular_vel_;

  // --- ROS интерфейсы ---
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_img_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr standoff_goal_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr charger_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr us_left_sub_, us_right_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr dock_srv_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;

  // --- Состояние (защищено mutex_) ---
  mutable std::mutex mutex_;
  geometry_msgs::msg::PoseStamped charger_pose_;
  bool have_charger_pose_ = false;
  sensor_msgs::msg::Image::SharedPtr last_image_;
  double fx_ = 466.0;
  float us_range_[2] = {std::numeric_limits<float>::infinity(),
                        std::numeric_limits<float>::infinity()};
  rclcpp::Time us_stamp_[2];

  std::atomic<bool> active_{false};

  cv::Ptr<cv::aruco::Dictionary> aruco_dict_ =
    cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
  cv::Ptr<cv::aruco::DetectorParameters> aruco_params_ =
    cv::aruco::DetectorParameters::create();
};

}  // namespace autonomous_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<autonomous_navigation::DockController>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
