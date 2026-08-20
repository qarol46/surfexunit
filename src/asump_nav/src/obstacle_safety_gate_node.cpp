#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/time.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>

using namespace std::chrono_literals;

class ObstacleSafetyGate : public rclcpp::Node
{
public:
  ObstacleSafetyGate()
  : Node("obstacle_safety_gate")
  {
    // ==================== Основные топики ====================
    cmd_vel_nav_topic_ = declare_parameter<std::string>("cmd_vel_nav_topic", "/cmd_vel_nav");
    cmd_vel_safe_topic_ = declare_parameter<std::string>("cmd_vel_safe_topic", "/cmd_vel_safe");

    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    map_topic_ = declare_parameter<std::string>("map_topic", "/map_base");
    estop_topic_ = declare_parameter<std::string>("estop_topic", "/estop");

    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    global_frame_ = declare_parameter<std::string>("global_frame", "map");

    // ==================== Пороги препятствий ====================
    outer_radius_ = declare_parameter<double>("outer_radius", 1.0);
    inner_radius_ = declare_parameter<double>("inner_radius", 0.35);
    min_speed_factor_ = declare_parameter<double>("min_speed_factor", 0.25);

    // Сектор контроля.
    // 360 — контролируем всё вокруг.
    // 180 — только передняя полусфера.
    // 120 — только фронтальный сектор.
    sector_angle_deg_ = declare_parameter<double>("sector_angle_deg", 180.0);

    // ==================== Фильтрация по карте ====================
    // Если карта не получена:
    // true  -> считать все препятствия динамическими, то есть реагировать
    // false -> не реагировать, пока нет карты
    fail_safe_without_map_ = declare_parameter<bool>("fail_safe_without_map", true);

    // Неизвестные клетки карты (-1):
    // true  -> считать потенциальным препятствием
    // false -> игнорировать
    treat_unknown_as_dynamic_ = declare_parameter<bool>("treat_unknown_as_dynamic", true);

    // Стоимость, начиная с которой клетка карты считается известным препятствием.
    // Для обычной карты: 90
    // Для inflated-карты часто лучше ставить 1, чтобы игнорировать инфляцию.
    known_obstacle_cost_threshold_ = declare_parameter<int>("known_obstacle_cost_threshold",90);

    // Сколько секунд держать состояние stop/slow после исчезновения препятствия.
    hold_sec_ = declare_parameter<double>("hold_sec", 0.5);

    // ==================== Safety-параметры ====================
    use_estop_ = declare_parameter<bool>("use_estop", true);
    enable_obstacle_detection_ = declare_parameter<bool>("enable_obstacle_detection", true);

    cmd_timeout_sec_ = declare_parameter<double>("cmd_timeout_sec", 0.5);
    scan_timeout_sec_ = declare_parameter<double>("scan_timeout_sec", 1.0);

    fail_safe_on_tf_error_ = declare_parameter<bool>("fail_safe_on_tf_error", true);

    publish_rate_ = declare_parameter<double>("publish_rate", 20.0);

    // Замедлять ли угловую скорость вместе с линейной.
    scale_angular_ = declare_parameter<bool>("scale_angular", false);
    min_angular_factor_ = declare_parameter<double>("min_angular_factor", 0.3);

    // ==================== Валидация параметров ====================
    if (outer_radius_ <= 0.0) {
      outer_radius_ = 1.0;
    }

    if (inner_radius_ < 0.0) {
      inner_radius_ = 0.0;
    }

    if (inner_radius_ >= outer_radius_) {
      inner_radius_ = 0.5 * outer_radius_;
      RCLCPP_WARN(
        get_logger(),
        "inner_radius >= outer_radius. Automatically set inner_radius = %.3f",
        inner_radius_);
    }

    min_speed_factor_ = std::clamp(min_speed_factor_, 0.0, 1.0);
    min_angular_factor_ = std::clamp(min_angular_factor_, 0.0, 1.0);

    if (sector_angle_deg_ < 0.0) {
      sector_angle_deg_ = 360.0;
    }

    if (sector_angle_deg_ > 360.0) {
      sector_angle_deg_ = 360.0;
    }

    if (cmd_timeout_sec_ <= 0.0) {
      cmd_timeout_sec_ = 0.5;
    }

    if (scan_timeout_sec_ <= 0.0) {
      scan_timeout_sec_ = 1.0;
    }

    if (publish_rate_ <= 0.0) {
      publish_rate_ = 20.0;
    }

    // ==================== TF ====================
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ==================== Publishers ====================
    cmd_safe_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      cmd_vel_safe_topic_,
      10);

    rclcpp::QoS debug_qos(1);
    debug_qos.transient_local();

    obstacle_factor_pub_ = create_publisher<std_msgs::msg::Float32>(
      "/obstacle_speed_factor",
      debug_qos);

    obstacle_stop_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/obstacle_stop",
      debug_qos);

    // ==================== Subscribers ====================
    cmd_nav_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_nav_topic_,
      10,
      std::bind(&ObstacleSafetyGate::cmdNavCallback, this, std::placeholders::_1));

    if (use_estop_) {
      estop_sub_ = create_subscription<std_msgs::msg::Bool>(
        estop_topic_,
        10,
        std::bind(&ObstacleSafetyGate::estopCallback, this, std::placeholders::_1));
    }

    if (enable_obstacle_detection_) {
      rclcpp::QoS map_qos(1);
      map_qos.transient_local();

      map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic_,
        map_qos,
        std::bind(&ObstacleSafetyGate::mapCallback, this, std::placeholders::_1));

      scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&ObstacleSafetyGate::scanCallback, this, std::placeholders::_1));
    }

    // ==================== Таймер публикации ====================
    auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ObstacleSafetyGate::timerCallback, this));

    RCLCPP_INFO(get_logger(), "ObstacleSafetyGate started.");
    RCLCPP_INFO(get_logger(), "cmd_vel_nav topic: %s", cmd_vel_nav_topic_.c_str());
    RCLCPP_INFO(get_logger(), "cmd_vel_safe topic: %s", cmd_vel_safe_topic_.c_str());
    RCLCPP_INFO(get_logger(), "scan topic: %s", scan_topic_.c_str());
    RCLCPP_INFO(get_logger(), "map topic: %s", map_topic_.c_str());
    RCLCPP_INFO(get_logger(), "estop topic: %s", use_estop_ ? estop_topic_.c_str() : "disabled");
    RCLCPP_INFO(
      get_logger(),
      "inner_radius = %.3f m, outer_radius = %.3f m, min_speed_factor = %.3f",
      inner_radius_, outer_radius_, min_speed_factor_);
  }

private:
  // ==================== Callbacks ====================

  void cmdNavCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);

    last_nav_cmd_ = *msg;
    last_nav_time_ = now();
    has_nav_cmd_ = true;

    publishSafeCmdUnlocked();
  }

  void estopCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);

    estop_active_ = msg->data;

    if (estop_active_) {
      RCLCPP_WARN(get_logger(), "E-STOP active inside safety gate.");
    } else {
      RCLCPP_INFO(get_logger(), "E-STOP released inside safety gate.");
    }

    publishSafeCmdUnlocked();
  }

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);

    map_ = *msg;
    has_map_ = true;

    if (!msg->header.frame_id.empty()) {
      map_frame_ = msg->header.frame_id;
    } else {
      map_frame_ = global_frame_;
    }

    RCLCPP_INFO_ONCE(
      get_logger(),
      "Map received: %dx%d, resolution = %.3f, frame = %s",
      map_.info.width, map_.info.height,
      map_.info.resolution, map_frame_.c_str());
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    if (!enable_obstacle_detection_) {
      return;
    }

    if (scan->ranges.empty()) {
      return;
    }

    const std::string scan_frame = scan->header.frame_id;

    if (scan_frame.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "LaserScan has empty frame_id.");
      return;
    }

    std::string map_frame;

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      map_frame = map_frame_;
    }

    geometry_msgs::msg::TransformStamped scan_to_base;
    geometry_msgs::msg::TransformStamped scan_to_map;

    try {
      scan_to_base = tf_buffer_->lookupTransform(
        base_frame_,
        scan_frame,
        tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      handleTfError("scan->base", ex.what());
      return;
    }

    try {
      scan_to_map = tf_buffer_->lookupTransform(
        map_frame,
        scan_frame,
        tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      handleTfError("scan->map", ex.what());
      return;
    }

    double min_dynamic_distance = std::numeric_limits<double>::infinity();

    const bool sector_enabled =
      sector_angle_deg_ > 0.0 && sector_angle_deg_ < 360.0;

    const double half_sector_rad =
      sector_angle_deg_ * M_PI / 360.0;

    {
      std::lock_guard<std::mutex> lock(state_mutex_);

      has_scan_ = true;
      last_scan_time_ = now();

      for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double range = scan->ranges[i];

        if (!std::isfinite(range)) {
          continue;
        }

        if (range < scan->range_min || range > scan->range_max) {
          continue;
        }

        // Быстрая отсечка по внешнему радиусу.
        if (range > outer_radius_) {
          continue;
        }

        double angle =
          scan->angle_min +
          static_cast<double>(i) * scan->angle_increment;

        double x_scan = range * std::cos(angle);
        double y_scan = range * std::sin(angle);

        // Расстояние и сектор в системе робота.
        double x_base = 0.0;
        double y_base = 0.0;
        transformPoint2D(scan_to_base, x_scan, y_scan, x_base, y_base);

        double distance = std::hypot(x_base, y_base);

        if (distance > outer_radius_) {
          continue;
        }

        if (sector_enabled) {
          double point_angle = std::atan2(y_base, x_base);

          if (std::fabs(point_angle) > half_sector_rad) {
            continue;
          }
        }

        // Позиция в карте для сравнения со статической картой.
        double x_map = 0.0;
        double y_map = 0.0;
        transformPoint2D(scan_to_map, x_scan, y_scan, x_map, y_map);

        // Если это известное препятствие по карте — игнорируем.
        if (isKnownObstacleUnlocked(x_map, y_map)) {
          continue;
        }

        if (distance < min_dynamic_distance) {
          min_dynamic_distance = distance;
        }
      }

      updateObstacleStateUnlocked(min_dynamic_distance);
      publishDebugUnlocked();
      publishSafeCmdUnlocked();
    }
  }

  void timerCallback()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    publishSafeCmdUnlocked();
  }

  // ==================== Safety publishing ====================

  void publishSafeCmdUnlocked()
  {
    auto current_time = now();

    geometry_msgs::msg::Twist out;

    bool nav_timeout =
      !has_nav_cmd_ ||
      (current_time - last_nav_time_).seconds() > cmd_timeout_sec_;

    bool scan_timeout =
      enable_obstacle_detection_ &&
      (!has_scan_ || (current_time - last_scan_time_).seconds() > scan_timeout_sec_);

    bool blocked =
      estop_active_ ||
      active_stop_ ||
      current_factor_ <= 0.0 ||
      scan_timeout;

    if (!nav_timeout && !blocked) {
      out = last_nav_cmd_;

      if (current_factor_ < 1.0) {
        out.linear.x *= current_factor_;

        if (scale_angular_) {
          double angular_factor =
            std::max(min_angular_factor_, current_factor_);

          out.angular.z *= angular_factor;
        }
      }
    } else {
      out.linear.x = 0.0;
      out.angular.z = 0.0;
    }

    cmd_safe_pub_->publish(out);
  }

  void publishDebugUnlocked()
  {
    std_msgs::msg::Float32 factor_msg;
    factor_msg.data = static_cast<float>(current_factor_);
    obstacle_factor_pub_->publish(factor_msg);

    std_msgs::msg::Bool stop_msg;
    stop_msg.data = active_stop_;
    obstacle_stop_pub_->publish(stop_msg);
  }

  // ==================== Obstacle state ====================

  void updateObstacleStateUnlocked(double min_distance)
  {
    if (!enable_obstacle_detection_) {
      active_stop_ = false;
      active_slow_ = false;
      current_factor_ = 1.0;
      return;
    }

    auto current_time = now();

    bool raw_stop =
      std::isfinite(min_distance) &&
      min_distance <= inner_radius_;

    bool raw_slow =
      std::isfinite(min_distance) &&
      min_distance > inner_radius_ &&
      min_distance <= outer_radius_;

    if (raw_stop) {
      active_stop_ = true;
      active_slow_ = false;
      current_factor_ = 0.0;
      last_active_distance_ = min_distance;

      stop_hold_until_ =
        current_time + rclcpp::Duration::from_seconds(hold_sec_);

      slow_hold_until_ =
        current_time + rclcpp::Duration::from_seconds(hold_sec_);
    } else if (raw_slow) {
      active_stop_ = false;
      active_slow_ = true;
      last_active_distance_ = min_distance;

      current_factor_ = computeFactorUnlocked(min_distance);

      slow_hold_until_ =
        current_time + rclcpp::Duration::from_seconds(hold_sec_);
    } else {
      // Препятствие исчезло, но держим состояние короткое время,
      // чтобы избежать дребезга.
      if (current_time < stop_hold_until_) {
        active_stop_ = true;
        active_slow_ = false;
        current_factor_ = 0.0;
      } else if (current_time < slow_hold_until_) {
        active_stop_ = false;
        active_slow_ = true;

        if (current_factor_ <= 0.0 || current_factor_ > 1.0) {
          current_factor_ = min_speed_factor_;
        }
      } else {
        active_stop_ = false;
        active_slow_ = false;
        current_factor_ = 1.0;
        last_active_distance_ = std::numeric_limits<double>::infinity();
      }
    }
  }

  double computeFactorUnlocked(double distance) const
  {
    if (distance <= inner_radius_) {
      return 0.0;
    }

    if (distance >= outer_radius_) {
      return 1.0;
    }

    double factor =
      (distance - inner_radius_) /
      (outer_radius_ - inner_radius_);

    return std::clamp(factor, min_speed_factor_, 1.0);
  }

  // ==================== Map helpers ====================

  bool isKnownObstacleUnlocked(double wx, double wy)
  {
    if (!has_map_) {
      // Если карты нет:
      // fail_safe_without_map_ == true  -> считать препятствие динамическим
      // fail_safe_without_map_ == false -> игнорировать
      return !fail_safe_without_map_;
    }

    int mx = 0;
    int my = 0;

    if (!worldToMapUnlocked(wx, wy, mx, my)) {
      // Вне карты. Чтобы не ловить ложные срабатывания на границе,
      // считаем это известной областью.
      return true;
    }

    size_t idx = static_cast<size_t>(my) *
                 static_cast<size_t>(map_.info.width) +
                 static_cast<size_t>(mx);

    int8_t cost = map_.data[idx];

    if (cost < 0) {
      // unknown cell
      return !treat_unknown_as_dynamic_;
    }

    return cost >= known_obstacle_cost_threshold_;
  }

  bool worldToMapUnlocked(double wx, double wy, int & mx, int & my) const
  {
    if (!has_map_) {
      return false;
    }

    double res = map_.info.resolution;

    if (res <= 0.0) {
      return false;
    }

    double gx = (wx - map_.info.origin.position.x) / res;
    double gy = (wy - map_.info.origin.position.y) / res;

    mx = static_cast<int>(std::floor(gx));
    my = static_cast<int>(std::floor(gy));

    if (
      mx < 0 ||
      my < 0 ||
      mx >= static_cast<int>(map_.info.width) ||
      my >= static_cast<int>(map_.info.height))
    {
      return false;
    }

    return true;
  }

  // ==================== TF helpers ====================

  void handleTfError(const std::string & transform_name, const char * what)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "TF error %s: %s", transform_name.c_str(), what);

    if (!fail_safe_on_tf_error_) {
      return;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);

    active_stop_ = true;
    active_slow_ = false;
    current_factor_ = 0.0;

    publishDebugUnlocked();
    publishSafeCmdUnlocked();
  }

  double quatToYaw(const geometry_msgs::msg::Quaternion & q) const
  {
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  void transformPoint2D(
    const geometry_msgs::msg::TransformStamped & tf,
    double x,
    double y,
    double & out_x,
    double & out_y) const
  {
    double yaw = quatToYaw(tf.transform.rotation);

    double cos_yaw = std::cos(yaw);
    double sin_yaw = std::sin(yaw);

    out_x =
      tf.transform.translation.x +
      x * cos_yaw - y * sin_yaw;

    out_y =
      tf.transform.translation.y +
      x * sin_yaw + y * cos_yaw;
  }

  // ==================== Members ====================

  std::string cmd_vel_nav_topic_;
  std::string cmd_vel_safe_topic_;

  std::string scan_topic_;
  std::string map_topic_;
  std::string estop_topic_;

  std::string base_frame_;
  std::string global_frame_;
  std::string map_frame_ = "map";

  double outer_radius_;
  double inner_radius_;
  double min_speed_factor_;
  double sector_angle_deg_;

  bool fail_safe_without_map_;
  bool treat_unknown_as_dynamic_;
  int known_obstacle_cost_threshold_;

  double hold_sec_;

  bool use_estop_;
  bool enable_obstacle_detection_;

  double cmd_timeout_sec_;
  double scan_timeout_sec_;

  bool fail_safe_on_tf_error_;

  double publish_rate_;

  bool scale_angular_;
  double min_angular_factor_;

  std::mutex state_mutex_;

  geometry_msgs::msg::Twist last_nav_cmd_;
  bool has_nav_cmd_ = false;
  rclcpp::Time last_nav_time_{0, 0, RCL_ROS_TIME};

  bool has_scan_ = false;
  rclcpp::Time last_scan_time_{0, 0, RCL_ROS_TIME};

  bool has_map_ = false;
  nav_msgs::msg::OccupancyGrid map_;

  bool estop_active_ = false;

  bool active_stop_ = false;
  bool active_slow_ = false;
  double current_factor_ = 1.0;

  double last_active_distance_ = std::numeric_limits<double>::infinity();

  rclcpp::Time stop_hold_until_{0, 0, RCL_ROS_TIME};
  rclcpp::Time slow_hold_until_{0, 0, RCL_ROS_TIME};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_nav_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_safe_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr obstacle_factor_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr obstacle_stop_pub_;

  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleSafetyGate>());
  rclcpp::shutdown();
  return 0;
}