#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <std_srvs/srv/trigger.hpp>
#include <asump_localization/srv/get_waypoint.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

using std::placeholders::_1;
using std::placeholders::_2;

using Trigger = std_srvs::srv::Trigger;
using GetWaypoint = asump_localization::srv::GetWaypoint;

class MissionPlanner : public rclcpp::Node
{
public:
  MissionPlanner()
  : Node("mission_planner")
  {
    // ==================== Параметры ====================
    map_topic_ = declare_parameter<std::string>("map_topic", "map_inflated");
    global_frame_ = declare_parameter<std::string>("global_frame", "map");

    robot_width_ = declare_parameter<double>("robot_width", 0.76);
    pass_overlap_ = declare_parameter<double>("pass_overlap", 0.0);
    line_spacing_ = declare_parameter<double>("line_spacing", 0.2);
    clearance_radius_ = declare_parameter<double>("clearance_radius", 0.8);

    min_segment_length_ = declare_parameter<double>("min_segment_length", 0.5);

    // Оставлен для совместимости со старыми launch-файлами.
    // В прямоугольной декомпозиции он сейчас не используется.
    max_gap_ = declare_parameter<double>("max_gap", 0.5);

    occupied_threshold_ = declare_parameter<int>("occupied_threshold", 20);
    treat_unknown_as_occupied_ = declare_parameter<bool>("treat_unknown_as_occupied", true);

    start_service_name_ = declare_parameter<std::string>(
      "start_service_name", "/mission/start");

    waypoint_service_name_ = declare_parameter<std::string>(
      "waypoint_service_name", "/mission/get_waypoint");

    if (line_spacing_ <= 0.0) {
      line_spacing_ = robot_width_ * (1.0 - pass_overlap_);
      if (line_spacing_ <= 0.0) {
        line_spacing_ = 0.3;
      }
    }

    if (clearance_radius_ <= 0.0) {
      clearance_radius_ = robot_width_ * 0.5 + 0.05;
    }

    // ==================== Publishers ====================
    rclcpp::QoS pub_qos(1);
    pub_qos.transient_local();

    key_points_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
      "mission/key_points",
      pub_qos);

    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "mission/key_points_markers",
      pub_qos);

    // ==================== Subscriber ====================
    rclcpp::QoS sub_qos(1);
    sub_qos.transient_local();

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_,
      sub_qos,
      std::bind(&MissionPlanner::mapCallback, this, _1));

    // ==================== Services ====================
    start_srv_ = create_service<Trigger>(
      start_service_name_,
      std::bind(&MissionPlanner::startServiceCallback, this, _1, _2));

    waypoint_srv_ = create_service<GetWaypoint>(
      waypoint_service_name_,
      std::bind(&MissionPlanner::waypointServiceCallback, this, _1, _2));

    RCLCPP_INFO(get_logger(), "Mission planner started.");
    RCLCPP_INFO(get_logger(), "Map topic: %s", map_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Start service: %s", start_service_name_.c_str());
    RCLCPP_INFO(get_logger(), "Waypoint service: %s", waypoint_service_name_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "line_spacing = %.3f m, clearance_radius = %.3f m",
      line_spacing_, clearance_radius_);
  }

private:
  struct Segment
  {
    geometry_msgs::msg::Pose start;
    geometry_msgs::msg::Pose end;
  };

  struct Rect
  {
    int x = 0;
    int y = 0;
    int w = 0;
    int h = 0;
  };

  // ==================== Map callback ====================

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);

    bool first_map = !has_map_;

    map_ = *msg;
    has_map_ = true;

    frame_id_ = map_.header.frame_id.empty()
      ? global_frame_
      : map_.header.frame_id;

    if (first_map) {
      RCLCPP_INFO(
        get_logger(),
        "Received first map: %dx%d, resolution = %.3f, frame = %s",
        map_.info.width, map_.info.height,
        map_.info.resolution, frame_id_.c_str());
    } else {
      RCLCPP_DEBUG(get_logger(), "Map updated.");
    }
  }

  // ==================== Services ====================

  void startServiceCallback(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (!has_map_) {
      response->success = false;
      response->message = "No map received yet";
      return;
    }

    if (
      map_.info.width == 0 ||
      map_.info.height == 0 ||
      map_.info.resolution <= 0.0)
    {
      response->success = false;
      response->message = "Empty or invalid map";
      return;
    }

    buildSafeGrid();

    auto rectangles = decomposeRectangles();
    auto ordered_rectangles = orderRectangles(rectangles);

    generateCoverage(ordered_rectangles);

    current_index_ = -1;

    publishVisualization();

    response->success = true;
    response->message =
      "Generated " + std::to_string(waypoints_.size()) +
      " waypoints from " + std::to_string(ordered_rectangles.size()) +
      " rectangles";

    RCLCPP_INFO(
      get_logger(),
      "Mission generated: %zu waypoints, %zu rectangles.",
      waypoints_.size(), ordered_rectangles.size());
  }

  void waypointServiceCallback(
    const std::shared_ptr<GetWaypoint::Request> request,
    std::shared_ptr<GetWaypoint::Response> response)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);

    response->success = false;
    response->finished = false;
    response->index = current_index_;
    response->total = static_cast<int>(waypoints_.size());

    if (waypoints_.empty()) {
      response->finished = true;
      response->success = false;
      return;
    }

    if (request->advance) {
      if (current_index_ + 1 < static_cast<int>(waypoints_.size())) {
        ++current_index_;
      } else {
        response->finished = true;
        response->success = false;
        response->index = current_index_;
        return;
      }
    } else {
      // Если запрашивают текущую точку до первого advance,
      // даём первую точку.
      if (current_index_ < 0) {
        current_index_ = 0;
      }
    }

    if (current_index_ < 0 || current_index_ >= static_cast<int>(waypoints_.size())) {
      response->finished = true;
      response->success = false;
      return;
    }

    response->success = true;
    response->finished = false;
    response->index = current_index_;
    response->total = static_cast<int>(waypoints_.size());

    response->pose.header.frame_id = frame_id_;
    response->pose.header.stamp = now();
    response->pose.pose = waypoints_[current_index_];

    RCLCPP_INFO(
      get_logger(),
      "Providing waypoint %d/%d: (%.2f, %.2f)",
      current_index_ + 1,
      static_cast<int>(waypoints_.size()),
      response->pose.pose.position.x,
      response->pose.pose.position.y);
  }

  // ==================== Grid helpers ====================

  size_t index(int x, int y) const
  {
    return static_cast<size_t>(y) *
           static_cast<size_t>(map_.info.width) +
           static_cast<size_t>(x);
  }

  bool inMap(int x, int y) const
  {
    return x >= 0 &&
           y >= 0 &&
           x < static_cast<int>(map_.info.width) &&
           y < static_cast<int>(map_.info.height);
  }

  bool isBlocked(int x, int y) const
  {
    if (!inMap(x, y)) {
      return true;
    }

    const auto cost = map_.data[index(x, y)];

    if (cost < 0) {
      return treat_unknown_as_occupied_;
    }

    return cost >= occupied_threshold_;
  }

  void buildSafeGrid()
  {
    const int w = static_cast<int>(map_.info.width);
    const int h = static_cast<int>(map_.info.height);
    const double res = map_.info.resolution;

    safe_grid_.assign(static_cast<size_t>(w) * static_cast<size_t>(h), false);

    std::vector<char> blocked(
      static_cast<size_t>(w) * static_cast<size_t>(h),
      false);

    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        blocked[index(x, y)] = isBlocked(x, y);
      }
    }

    int radius_cells = static_cast<int>(std::ceil(clearance_radius_ / res));
    if (radius_cells < 0) {
      radius_cells = 0;
    }

    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        if (blocked[index(x, y)]) {
          continue;
        }

        bool ok = true;

        for (int dy = -radius_cells; dy <= radius_cells && ok; ++dy) {
          for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
            if (dx * dx + dy * dy > radius_cells * radius_cells) {
              continue;
            }

            int nx = x + dx;
            int ny = y + dy;

            if (!inMap(nx, ny) || blocked[index(nx, ny)]) {
              ok = false;
              break;
            }
          }
        }

        safe_grid_[index(x, y)] = ok;
      }
    }
  }

  bool isFreeUncovered(
    int x,
    int y,
    const std::vector<char> & covered) const
  {
    if (!inMap(x, y)) {
      return false;
    }

    const auto idx = index(x, y);

    return safe_grid_[idx] && !covered[idx];
  }

  // ==================== Rectangle decomposition ====================

  int horizontalRun(
    int x,
    int y,
    const std::vector<char> & covered) const
  {
    int len = 0;

    while (isFreeUncovered(x + len, y, covered)) {
      ++len;
    }

    return len;
  }

  int verticalRun(
    int x,
    int y,
    const std::vector<char> & covered) const
  {
    int len = 0;

    while (isFreeUncovered(x, y + len, covered)) {
      ++len;
    }

    return len;
  }

  bool canExpandDown(
    const Rect & r,
    const std::vector<char> & covered) const
  {
    int ny = r.y + r.h;

    if (ny >= static_cast<int>(map_.info.height)) {
      return false;
    }

    for (int dx = 0; dx < r.w; ++dx) {
      if (!isFreeUncovered(r.x + dx, ny, covered)) {
        return false;
      }
    }

    return true;
  }

  bool canExpandRight(
    const Rect & r,
    const std::vector<char> & covered) const
  {
    int nx = r.x + r.w;

    if (nx >= static_cast<int>(map_.info.width)) {
      return false;
    }

    for (int dy = 0; dy < r.h; ++dy) {
      if (!isFreeUncovered(nx, r.y + dy, covered)) {
        return false;
      }
    }

    return true;
  }

  void markRect(
    const Rect & r,
    std::vector<char> & covered) const
  {
    for (int yy = r.y; yy < r.y + r.h; ++yy) {
      for (int xx = r.x; xx < r.x + r.w; ++xx) {
        covered[index(xx, yy)] = true;
      }
    }
  }

  std::vector<Rect> decomposeRectangles()
  {
    std::vector<Rect> rectangles;

    const int w = static_cast<int>(map_.info.width);
    const int h = static_cast<int>(map_.info.height);

    std::vector<char> covered(
      static_cast<size_t>(w) * static_cast<size_t>(h),
      false);

    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        if (!isFreeUncovered(x, y, covered)) {
          continue;
        }

        int h_run = horizontalRun(x, y, covered);
        int v_run = verticalRun(x, y, covered);

        Rect rect;
        rect.x = x;
        rect.y = y;
        rect.w = 1;
        rect.h = 1;

        if (h_run >= v_run) {
          rect.w = h_run;
          rect.h = 1;

          while (canExpandDown(rect, covered)) {
            ++rect.h;
          }
        } else {
          rect.h = v_run;
          rect.w = 1;

          while (canExpandRight(rect, covered)) {
            ++rect.w;
          }
        }

        markRect(rect, covered);
        rectangles.push_back(rect);
      }
    }

    return rectangles;
  }

  double rectCenterX(const Rect & r) const
  {
    return static_cast<double>(r.x) + 0.5 * static_cast<double>(r.w);
  }

  double rectCenterY(const Rect & r) const
  {
    return static_cast<double>(r.y) + 0.5 * static_cast<double>(r.h);
  }

  double rectDistance2(const Rect & a, const Rect & b) const
  {
    double dx = rectCenterX(a) - rectCenterX(b);
    double dy = rectCenterY(a) - rectCenterY(b);

    return dx * dx + dy * dy;
  }

  std::vector<Rect> orderRectangles(const std::vector<Rect> & rectangles)
  {
    std::vector<Rect> ordered;

    if (rectangles.empty()) {
      return ordered;
    }

    std::vector<Rect> remaining = rectangles;

    // Стартовый прямоугольник: самый нижний/левый по center.
    std::sort(
      remaining.begin(),
      remaining.end(),
      [this](const Rect & a, const Rect & b) {
        double ay = rectCenterY(a);
        double by = rectCenterY(b);

        if (std::fabs(ay - by) > 1e-6) {
          return ay < by;
        }

        return rectCenterX(a) < rectCenterX(b);
      });

    Rect current = remaining.front();
    remaining.erase(remaining.begin());
    ordered.push_back(current);

    while (!remaining.empty()) {
      double best_dist = std::numeric_limits<double>::max();
      size_t best_idx = 0;

      for (size_t i = 0; i < remaining.size(); ++i) {
        double d = rectDistance2(current, remaining[i]);

        if (d < best_dist) {
          best_dist = d;
          best_idx = i;
        }
      }

      current = remaining[best_idx];
      remaining.erase(remaining.begin() + best_idx);
      ordered.push_back(current);
    }

    return ordered;
  }

  // ==================== Coverage generation ====================

  geometry_msgs::msg::Quaternion quatFromYaw(double yaw) const
  {
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
    return q;
  }

  std::vector<double> samplePositions(
    double min_value,
    double max_value,
    double spacing) const
  {
    std::vector<double> result;

    const double eps = 1e-3;
    const double length = max_value - min_value;

    if (length <= spacing * 0.5) {
      result.push_back(0.5 * (min_value + max_value));
      return result;
    }

    double p = min_value + spacing * 0.5;

    while (p <= max_value - spacing * 0.5 + eps) {
      result.push_back(p);
      p += spacing;
    }

    if (result.empty()) {
      result.push_back(0.5 * (min_value + max_value));
    }

    return result;
  }

  void addLine(
    const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & end)
  {
    double dx = end.position.x - start.position.x;
    double dy = end.position.y - start.position.y;
    double length = std::hypot(dx, dy);

    if (length < min_segment_length_) {
      return;
    }

    waypoints_.push_back(start);
    waypoints_.push_back(end);
    segments_.push_back(Segment{start, end});
  }

  void addRectCoverage(const Rect & rect)
  {
    const double res = map_.info.resolution;

    const double x_min =
      map_.info.origin.position.x +
      (static_cast<double>(rect.x) + 0.5) * res;

    const double x_max =
      map_.info.origin.position.x +
      (static_cast<double>(rect.x + rect.w - 1) + 0.5) * res;

    const double y_min =
      map_.info.origin.position.y +
      (static_cast<double>(rect.y) + 0.5) * res;

    const double y_max =
      map_.info.origin.position.y +
      (static_cast<double>(rect.y + rect.h - 1) + 0.5) * res;

    const double width_m = static_cast<double>(rect.w) * res;
    const double height_m = static_cast<double>(rect.h) * res;

    if (width_m >= height_m) {
      // Горизонтальные проходы вдоль X.
      auto y_values = samplePositions(y_min, y_max, line_spacing_);

      for (double y : y_values) {
        geometry_msgs::msg::Pose start;
        geometry_msgs::msg::Pose end;

        start.position.z = 0.0;
        end.position.z = 0.0;

        start.position.y = y;
        end.position.y = y;

        if (snake_forward_) {
          start.position.x = x_min;
          end.position.x = x_max;

          start.orientation = quatFromYaw(0.0);
          end.orientation = quatFromYaw(0.0);
        } else {
          start.position.x = x_max;
          end.position.x = x_min;

          start.orientation = quatFromYaw(M_PI);
          end.orientation = quatFromYaw(M_PI);
        }

        addLine(start, end);
        snake_forward_ = !snake_forward_;
      }
    } else {
      // Вертикальные проходы вдоль Y.
      auto x_values = samplePositions(x_min, x_max, line_spacing_);

      for (double x : x_values) {
        geometry_msgs::msg::Pose start;
        geometry_msgs::msg::Pose end;

        start.position.z = 0.0;
        end.position.z = 0.0;

        start.position.x = x;
        end.position.x = x;

        if (snake_forward_) {
          start.position.y = y_min;
          end.position.y = y_max;

          start.orientation = quatFromYaw(M_PI / 2.0);
          end.orientation = quatFromYaw(M_PI / 2.0);
        } else {
          start.position.y = y_max;
          end.position.y = y_min;

          start.orientation = quatFromYaw(-M_PI / 2.0);
          end.orientation = quatFromYaw(-M_PI / 2.0);
        }

        addLine(start, end);
        snake_forward_ = !snake_forward_;
      }
    }
  }

  void generateCoverage(const std::vector<Rect> & rectangles)
  {
    waypoints_.clear();
    segments_.clear();

    snake_forward_ = true;

    for (const auto & rect : rectangles) {
      addRectCoverage(rect);
    }

    if (waypoints_.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "No coverage waypoints generated. Check clearance_radius, occupied_threshold and min_segment_length.");
    }
  }

  // ==================== Visualization ====================

  void publishVisualization()
  {
    key_points_.poses = waypoints_;
    key_points_.header.frame_id = frame_id_;
    key_points_.header.stamp = now();

    key_points_pub_->publish(key_points_);

    publishMarkers();
  }

  void publishMarkers()
  {
    visualization_msgs::msg::MarkerArray markers;

    visualization_msgs::msg::Marker delete_all;
    delete_all.header.frame_id = frame_id_;
    delete_all.header.stamp = now();
    delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(delete_all);

    if (waypoints_.empty()) {
      markers_pub_->publish(markers);
      return;
    }

    // Линия порядка объезда
    visualization_msgs::msg::Marker line;
    line.header.frame_id = frame_id_;
    line.header.stamp = now();
    line.ns = "coverage_order";
    line.id = 1;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.pose.orientation.w = 1.0;
    line.scale.x = 0.02;
    line.color.r = 0.0;
    line.color.g = 1.0;
    line.color.b = 0.0;
    line.color.a = 1.0;

    for (const auto & p : waypoints_) {
      line.points.push_back(p.position);
    }

    markers.markers.push_back(line);

    // Точки
    visualization_msgs::msg::Marker spheres;
    spheres.header.frame_id = frame_id_;
    spheres.header.stamp = now();
    spheres.ns = "coverage_points";
    spheres.id = 2;
    spheres.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    spheres.action = visualization_msgs::msg::Marker::ADD;
    spheres.pose.orientation.w = 1.0;

    spheres.scale.x = 0.07;
    spheres.scale.y = 0.07;
    spheres.scale.z = 0.07;

    spheres.color.r = 1.0;
    spheres.color.g = 0.2;
    spheres.color.b = 0.2;
    spheres.color.a = 1.0;

    for (const auto & p : waypoints_) {
      spheres.points.push_back(p.position);
    }

    markers.markers.push_back(spheres);

    // Стрелки направлений
    int id = 10;

    for (const auto & seg : segments_) {
      visualization_msgs::msg::Marker arrow;
      arrow.header.frame_id = frame_id_;
      arrow.header.stamp = now();
      arrow.ns = "coverage_segments";
      arrow.id = id++;
      arrow.type = visualization_msgs::msg::Marker::ARROW;
      arrow.action = visualization_msgs::msg::Marker::ADD;
      arrow.pose.orientation.w = 1.0;

      arrow.points.push_back(seg.start.position);
      arrow.points.push_back(seg.end.position);

      arrow.scale.x = 0.02;
      arrow.scale.y = 0.06;
      arrow.scale.z = 0.10;

      arrow.color.r = 0.2;
      arrow.color.g = 0.6;
      arrow.color.b = 1.0;
      arrow.color.a = 0.9;

      markers.markers.push_back(arrow);
    }

    markers_pub_->publish(markers);
  }

  // ==================== Parameters ====================
  std::string map_topic_;
  std::string global_frame_;

  double robot_width_;
  double pass_overlap_;
  double line_spacing_;
  double clearance_radius_;
  double min_segment_length_;
  double max_gap_;

  int occupied_threshold_;
  bool treat_unknown_as_occupied_;

  std::string start_service_name_;
  std::string waypoint_service_name_;

  // ==================== State ====================
  std::mutex state_mutex_;

  bool has_map_ = false;
  std::string frame_id_;

  nav_msgs::msg::OccupancyGrid map_;
  std::vector<bool> safe_grid_;

  std::vector<geometry_msgs::msg::Pose> waypoints_;
  std::vector<Segment> segments_;

  geometry_msgs::msg::PoseArray key_points_;

  int current_index_ = -1;
  bool snake_forward_ = true;

  // ==================== ROS ====================
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr key_points_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;

  rclcpp::Service<Trigger>::SharedPtr start_srv_;
  rclcpp::Service<GetWaypoint>::SharedPtr waypoint_srv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionPlanner>());
  rclcpp::shutdown();
  return 0;
}