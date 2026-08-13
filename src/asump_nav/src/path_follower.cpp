#include "asump_nav/path_follower.hpp"

#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <cmath>
#include <future>
#include <limits>
#include <string>
#include <thread>

namespace autonomous_navigation
{

PathFollower::PathFollower(const rclcpp::NodeOptions & options)
: Node("path_follower", options)
{
  // ==================== Параметры ====================
  max_linear_vel_ = declare_parameter<double>("max_linear_vel", 0.18);
  max_angular_vel_ = declare_parameter<double>("max_angular_vel", 2.0);

  // Новые параметры: минимальные скорости и мёртвая зона угла
  min_linear_vel_ = declare_parameter<double>("min_linear_vel", 0.1);
  min_angular_vel_ = declare_parameter<double>("min_angular_vel", 1.3);
  angle_dead_zone_ = declare_parameter<double>("angle_dead_zone", 0.15); // ~2 градуса

  min_approach_linear_vel_ = declare_parameter<double>("min_approach_linear_vel", 0.05);
  approach_velocity_scaling_dist_ = declare_parameter<double>("approach_velocity_scaling_dist", 1.0);

  base_lookahead_dist_ = declare_parameter<double>("base_lookahead_dist", 0.6);
  min_lookahead_dist_ = declare_parameter<double>("min_lookahead_dist", 0.3);
  max_lookahead_dist_ = declare_parameter<double>("max_lookahead_dist", 1.2);
  lookahead_time_ = declare_parameter<double>("lookahead_time", 1.5);

  goal_reached_tolerance_ = declare_parameter<double>("goal_reached_tolerance", 0.1);
  goal_yaw_tolerance_ = declare_parameter<double>("goal_yaw_tolerance", 0.25);

  curvature_velocity_scaling_ = declare_parameter<double>("curvature_velocity_scaling", 0.5);
  control_frequency_ = declare_parameter<double>("control_frequency", 20.0);
  planner_timeout_ = declare_parameter<double>("planner_timeout", 5.0);

  use_velocity_scaled_lookahead_ = declare_parameter<bool>("use_velocity_scaled_lookahead", true);
  use_regulated_linear_velocity_scaling_ = declare_parameter<bool>(
    "use_regulated_linear_velocity_scaling", true);

  robot_base_frame_ = declare_parameter<std::string>("robot_base_frame", "base_footprint");
  global_frame_ = declare_parameter<std::string>("global_frame", "map");

  planner_service_name_ = declare_parameter<std::string>(
    "planner_service_name", "get_path_to_point");

  // ==================== TF ====================
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // ==================== cmd_vel ====================
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // ==================== Nav status ====================
  rclcpp::QoS nav_status_qos(10);
  nav_status_qos.transient_local();

  nav_status_pub_ = create_publisher<std_msgs::msg::String>(
    "/path_follower/nav_status",
    nav_status_qos);

  // ==================== Клиент планировщика ====================
  planner_client_ = create_client<GetPathToPoint>(planner_service_name_);

  // ==================== Action-сервер ====================
  action_server_ = rclcpp_action::create_server<NavigateToPose>(
    this,
    "navigate_to_pose",
    std::bind(&PathFollower::handle_goal, this,
      std::placeholders::_1, std::placeholders::_2),
    std::bind(&PathFollower::handle_cancel, this, std::placeholders::_1),
    std::bind(&PathFollower::handle_accepted, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "PathFollower готов.");
  RCLCPP_INFO(get_logger(), "  Action: /navigate_to_pose");
  RCLCPP_INFO(get_logger(), "  Сервис планировщика: /%s", planner_service_name_.c_str());
  RCLCPP_INFO(get_logger(), "  Nav status topic: /path_follower/nav_status");

  RCLCPP_INFO(
    get_logger(),
    "  max_lin=%.2f, min_lin=%.2f, max_ang=%.2f, min_ang=%.2f, dead_zone=%.3f, lookahead=%.2f",
    max_linear_vel_, min_linear_vel_, max_angular_vel_, min_angular_vel_,
    angle_dead_zone_, base_lookahead_dist_);
}

// ==================== Action callbacks ====================

rclcpp_action::GoalResponse PathFollower::handle_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const NavigateToPose::Goal> goal)
{
  RCLCPP_INFO(get_logger(), "Принята цель: (%.2f, %.2f)",
    goal->pose.pose.position.x, goal->pose.pose.position.y);

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PathFollower::handle_cancel(
  const std::shared_ptr<GoalHandleNav> /*goal_handle*/)
{
  RCLCPP_INFO(get_logger(), "Получен запрос на отмену");

  geometry_msgs::msg::Twist stop;
  cmd_vel_pub_->publish(stop);

  publishNavStatus("CANCEL_REQUESTED");

  return rclcpp_action::CancelResponse::ACCEPT;
}

void PathFollower::handle_accepted(const std::shared_ptr<GoalHandleNav> goal_handle)
{
  std::thread{std::bind(&PathFollower::execute, this, std::placeholders::_1),
    goal_handle}.detach();
}

void PathFollower::execute(const std::shared_ptr<GoalHandleNav> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<NavigateToPose::Result>();

  publishNavStatus("PLANNING");

  // ШАГ 1: запрос пути у планировщика
  nav_msgs::msg::Path path;

  if (!plan_path(goal->pose, path)) {
    RCLCPP_ERROR(get_logger(), "Не удалось построить путь — цель отклонена");

    if (last_plan_error_.empty()) {
      publishNavStatus("PATH_NOT_FOUND");
    } else {
      publishNavStatus(last_plan_error_);
    }

    goal_handle->abort(result);
    return;
  }

  if (path.poses.empty()) {
    RCLCPP_ERROR(get_logger(), "Планировщик вернул пустой путь");
    publishNavStatus("EMPTY_PATH");
    goal_handle->abort(result);
    return;
  }

  RCLCPP_INFO(get_logger(), "Путь построен: %zu точек. Начинаю движение...",
    path.poses.size());

  publishNavStatus("MOVING");

  // ШАГ 2: следование по пути
  follow_path(goal_handle, path);
}

// ==================== Публикация статуса ====================

void PathFollower::publishNavStatus(const std::string & status)
{
  std_msgs::msg::String msg;
  msg.data = status;
  nav_status_pub_->publish(msg);

  RCLCPP_DEBUG(get_logger(), "Nav status: %s", status.c_str());
}

// ==================== Планирование через сервис ====================

bool PathFollower::plan_path(
  const geometry_msgs::msg::PoseStamped & goal,
  nav_msgs::msg::Path & out_path)
{
  last_plan_error_ = "UNKNOWN";

  // Ждём сервис
  if (!planner_client_->wait_for_service(
      std::chrono::seconds(static_cast<int>(planner_timeout_))))
  {
    RCLCPP_ERROR(get_logger(), "Сервис планировщика /%s недоступен",
      planner_service_name_.c_str());

    last_plan_error_ = "PLANNER_SERVICE_UNAVAILABLE";
    return false;
  }

  auto request = std::make_shared<GetPathToPoint::Request>();
  request->goal_pose = goal;

  RCLCPP_INFO(get_logger(), "Запрашиваю путь у планировщика...");

  auto future = planner_client_->async_send_request(request);

  // Ждём ответ с таймаутом
  auto status = future.wait_for(
    std::chrono::seconds(static_cast<int>(planner_timeout_)));

  if (status != std::future_status::ready) {
    RCLCPP_ERROR(get_logger(), "Таймаут при ожидании ответа планировщика");
    last_plan_error_ = "PLANNER_TIMEOUT";
    return false;
  }

  auto response = future.get();

  if (!response->success) {
    RCLCPP_ERROR(get_logger(), "Планировщик сообщил об ошибке");
    last_plan_error_ = "PLANNER_ERROR";
    return false;
  }

  out_path = response->path;
  last_plan_error_.clear();

  return true;
}

// ==================== Control loop ====================

void PathFollower::follow_path(
  const std::shared_ptr<GoalHandleNav> goal_handle,
  const nav_msgs::msg::Path & path)
{
  auto result = std::make_shared<NavigateToPose::Result>();
  auto feedback = std::make_shared<NavigateToPose::Feedback>();

  rclcpp::Rate rate(control_frequency_);

  double current_linear_vel = 0.0;

  // === ЛОКАЛЬНЫЕ ПАРАМЕТРЫ РЕГУЛИРОВАНИЯ ===
  const double MAX_ANGLE_BEFORE_STOP  = 0.3;   // рад (~35°)
  const double MAX_ANGLE_FOR_FULL_SPEED = 0.1; // рад (~9°)
  const double ROTATION_KP            = 2.5;   // коэффициент для разворота на месте
  const double STEERING_KP            = 1.8;   // коэффициент для корректировки в движении
  const bool   INVERT_ANGULAR         = false; // true, если робот крутится не туда

  // Вычисляем актуальные ограничения на основе полей класса
  const double max_linear = std::max(0.0, max_linear_vel_);
  const double max_angular = std::max(0.0, max_angular_vel_);
  const double min_linear = std::clamp(min_linear_vel_, 0.0, max_linear);
  const double min_angular = std::clamp(min_angular_vel_, 0.0, max_angular);
  const double dead_zone = std::max(0.0, angle_dead_zone_);

  // Лямбда для ограничения угловой скорости с поддержкой минимума и мёртвой зоны.
  // Убирает "провал в ноль", если ошибка больше dead_zone.
  auto limit_angular = [&](double raw, double err, double dz) -> double
  {
    if (std::fabs(err) <= dz) {
      return 0.0;
    }

    const double abs_raw = std::fabs(raw);
    double abs_cmd = abs_raw;

    if (abs_cmd < min_angular) {
      abs_cmd = min_angular;
    }

    abs_cmd = std::clamp(abs_cmd, min_angular, max_angular);

    const double sign_source = (abs_raw > 1e-6) ? raw : err;
    return std::copysign(abs_cmd, sign_source);
  };

  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      geometry_msgs::msg::Twist stop;
      cmd_vel_pub_->publish(stop);

      publishNavStatus("CANCELED");

      goal_handle->canceled(result);
      return;
    }

    geometry_msgs::msg::PoseStamped current_pose;

    try {
      current_pose = get_current_pose();
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "Не удалось получить позу: %s", e.what());
      rate.sleep();
      continue;
    }

    const auto & goal_pose = path.poses.back();

    double dist_to_goal = distance_2d(current_pose.pose.position,
                                      goal_pose.pose.position);

    feedback->distance_remaining = dist_to_goal;
    goal_handle->publish_feedback(feedback);

    // ====== Цель достигнута ======
    if (dist_to_goal < goal_reached_tolerance_) {
      double current_yaw = yaw_from_quat(current_pose.pose.orientation);
      double goal_yaw = yaw_from_quat(goal_pose.pose.orientation);
      double yaw_error = std::abs(normalize_angle(goal_yaw - current_yaw));

      bool no_yaw_requested =
        (goal_pose.pose.orientation.w > 0.99 &&
         std::abs(goal_pose.pose.orientation.z) < 1e-3);

      if (yaw_error < goal_yaw_tolerance_ || no_yaw_requested) {
        geometry_msgs::msg::Twist stop;
        cmd_vel_pub_->publish(stop);

        RCLCPP_INFO(get_logger(), "Цель достигнута!");
        publishNavStatus("REACHED");

        goal_handle->succeed(result);
        return;
      }

      geometry_msgs::msg::Twist cmd;
      double ang_err = normalize_angle(goal_yaw - current_yaw);

      // При довороте на цели dead_zone = 0.0, чтобы точно доехать до нужного угла.
      cmd.angular.z = limit_angular(ang_err * ROTATION_KP, ang_err, 0.0);

      if (INVERT_ANGULAR) {
        cmd.angular.z = -cmd.angular.z;
      }

      cmd_vel_pub_->publish(cmd);
      rate.sleep();
      continue;
    }

    // ====== Находим lookahead-точку ======
    double lookahead = compute_lookahead(current_linear_vel);
    auto lookahead_point = find_lookahead_point(current_pose, path, lookahead);

    // ====== Угол до lookahead в глобальной системе ======
    double dx_global = lookahead_point.pose.position.x - current_pose.pose.position.x;
    double dy_global = lookahead_point.pose.position.y - current_pose.pose.position.y;
    double angle_to_lookahead = std::atan2(dy_global, dx_global);

    double current_yaw = yaw_from_quat(current_pose.pose.orientation);
    double angle_error = normalize_angle(angle_to_lookahead - current_yaw);

    RCLCPP_DEBUG(get_logger(),
      "angle_to_lookahead=%.2f, current_yaw=%.2f, angle_error=%.2f",
      angle_to_lookahead, current_yaw, angle_error);

    // ====== ФАЗА РАЗВОРОТА: угол слишком большой ======
    if (std::abs(angle_error) > MAX_ANGLE_BEFORE_STOP) {
      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = 0.0;
      cmd.angular.z = limit_angular(
        angle_error * ROTATION_KP,
        angle_error,
        dead_zone);

      if (INVERT_ANGULAR) {
        cmd.angular.z = -cmd.angular.z;
      }

      cmd_vel_pub_->publish(cmd);
      current_linear_vel = 0.0;
      rate.sleep();
      continue;
    }

    // ====== ФАЗА ДВИЖЕНИЯ: угол в норме ======

    // Преобразуем lookahead в систему робота для Pure Pursuit
    geometry_msgs::msg::PoseStamped lookahead_in_robot;

    try {
      auto tf = tf_buffer_->lookupTransform(
        robot_base_frame_, global_frame_, tf2::TimePointZero);

      tf2::doTransform(lookahead_point, lookahead_in_robot, tf);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "TF error: %s", ex.what());
      rate.sleep();
      continue;
    }

    double lx = lookahead_in_robot.pose.position.x;
    double ly = lookahead_in_robot.pose.position.y;
    double L2 = lx * lx + ly * ly;

    double curvature = (L2 < 1e-6) ? 0.0 : (2.0 * ly / L2);

    // Линейная скорость
    double v = compute_linear_velocity(dist_to_goal, curvature, current_linear_vel);

    // Снижение скорости при большом угле
    double angle_factor = 1.0;

    if (std::abs(angle_error) > MAX_ANGLE_FOR_FULL_SPEED) {
      angle_factor = 1.0 -
        (std::abs(angle_error) - MAX_ANGLE_FOR_FULL_SPEED) /
        (MAX_ANGLE_BEFORE_STOP - MAX_ANGLE_FOR_FULL_SPEED);

      angle_factor = std::clamp(angle_factor, 0.0, 1.0);
    }

    v *= angle_factor;

    // Применяем минимальную линейную скорость, если движение разрешено
    if (v > 0.0) {
      v = std::max(v, min_linear);
    }

    v = std::clamp(v, 0.0, max_linear);

    // === УГЛОВАЯ СКОРОСТЬ ===
    double omega = 0.0;

    if (std::abs(angle_error) > dead_zone) {
      // Комбинируем Pure Pursuit (кривизна) и коррекцию по углу
      double omega_pursuit = v * curvature;
      double omega_correction = angle_error * STEERING_KP;

      // Берём среднее, чтобы не было резких движений
      omega = 0.5 * omega_pursuit + 0.5 * omega_correction;
    }

    // Если скорость почти 0, но угол есть — доворачиваем на месте
    if (v < 0.05 && std::abs(angle_error) > dead_zone) {
      omega = limit_angular(
        angle_error * ROTATION_KP,
        angle_error,
        dead_zone);
    } else {
      omega = limit_angular(omega, angle_error, dead_zone);
    }

    if (INVERT_ANGULAR) {
      omega = -omega;
    }

    current_linear_vel = v;

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = v;
    cmd.angular.z = omega;

    cmd_vel_pub_->publish(cmd);

    rate.sleep();
  }
}

// ==================== Pure pursuit helpers ====================

geometry_msgs::msg::PoseStamped PathFollower::get_current_pose()
{
  auto tf = tf_buffer_->lookupTransform(
    global_frame_, robot_base_frame_, tf2::TimePointZero);

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = global_frame_;
  pose.header.stamp = get_clock()->now();

  pose.pose.position.x = tf.transform.translation.x;
  pose.pose.position.y = tf.transform.translation.y;
  pose.pose.position.z = tf.transform.translation.z;
  pose.pose.orientation = tf.transform.rotation;

  return pose;
}

geometry_msgs::msg::PoseStamped PathFollower::find_lookahead_point(
  const geometry_msgs::msg::PoseStamped & current,
  const nav_msgs::msg::Path & path,
  double lookahead)
{
  size_t closest_idx = 0;
  double min_dist = std::numeric_limits<double>::max();

  for (size_t i = 0; i < path.poses.size(); ++i) {
    double d = distance_2d(current.pose.position, path.poses[i].pose.position);

    if (d < min_dist) {
      min_dist = d;
      closest_idx = i;
    }
  }

  for (size_t i = closest_idx; i < path.poses.size(); ++i) {
    double d = distance_2d(current.pose.position, path.poses[i].pose.position);

    if (d >= lookahead) {
      return path.poses[i];
    }
  }

  return path.poses.back();
}

double PathFollower::compute_linear_velocity(
  double dist_to_goal, double curvature, double current_vel)
{
  const double max_linear = std::max(0.0, max_linear_vel_);
  const double min_approach = std::clamp(min_approach_linear_vel_, 0.0, max_linear);
  const double min_linear = std::clamp(min_linear_vel_, 0.0, max_linear);
  const double lower_speed = std::max(min_approach, min_linear);

  double v = max_linear;

  if (approach_velocity_scaling_dist_ > 0.0 &&
    dist_to_goal < approach_velocity_scaling_dist_)
  {
    double scale = std::max(0.2, dist_to_goal / approach_velocity_scaling_dist_);
    v *= scale;
  }

  if (use_regulated_linear_velocity_scaling_) {
    double penalty = std::abs(curvature) * curvature_velocity_scaling_;
    v *= 1.0 / (1.0 + penalty);
  }

  // Нижняя граница скорости до ограничения ускорения
  if (v > 0.0) {
    v = std::max(v, lower_speed);
  }

  double max_accel = 0.5;
  double dt = (control_frequency_ > 0.0) ? (1.0 / control_frequency_) : 0.05;

  double dv = v - current_vel;

  if (std::abs(dv) > max_accel * dt) {
    v = current_vel + std::copysign(max_accel * dt, dv);
  }

  // После ограничения ускорения также не даём команде провалиться ниже минимума
  if (v > 0.0) {
    v = std::max(v, lower_speed);
  }

  return std::clamp(v, 0.0, max_linear);
}

double PathFollower::compute_lookahead(double linear_vel)
{
  if (!use_velocity_scaled_lookahead_) {
    return base_lookahead_dist_;
  }

  double la = base_lookahead_dist_ + linear_vel * lookahead_time_;
  return std::clamp(la, min_lookahead_dist_, max_lookahead_dist_);
}

// ==================== Utils ====================

double PathFollower::yaw_from_quat(const geometry_msgs::msg::Quaternion & q)
{
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double PathFollower::normalize_angle(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

double PathFollower::distance_2d(
  const geometry_msgs::msg::Point & a,
  const geometry_msgs::msg::Point & b)
{
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

}  // namespace autonomous_navigation