#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

using std::placeholders::_1;

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
    pass_overlap_ = declare_parameter<double>("pass_overlap", 0.30);

    // Если 0, посчитаем автоматически:
    // line_spacing = robot_width * (1 - overlap)
    line_spacing_ = declare_parameter<double>("line_spacing", 0.2);

    // Радиус безопасности вокруг точки.
    // Если 0, посчитаем как robot_width / 2 + запас.
    clearance_radius_ = declare_parameter<double>("clearance_radius", 0.8);

    // Минимальная длина свободного сегмента, чтобы он стал проходом.
    min_segment_length_ = declare_parameter<double>("min_segment_length", 0.7);

    // Максимальный разрыв, который можно "перепрыгнуть" внутри строки.
    // 0.0 — объединять только соседние свободные клетки.
    max_gap_ = declare_parameter<double>("max_gap", 0.5);

    // Клетки с cost >= occupied_threshold считаются занятыми.
    occupied_threshold_ = declare_parameter<int>("occupied_threshold", 20);

    // Unknown cells (-1) считать занятыми.
    treat_unknown_as_occupied_ = declare_parameter<bool>("treat_unknown_as_occupied", true);

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

    RCLCPP_INFO(get_logger(), "Mission planner started.");
    RCLCPP_INFO(get_logger(), "Map topic: %s", map_topic_.c_str());
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

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    map_ = *msg;

    frame_id_ = map_.header.frame_id.empty()
      ? global_frame_
      : map_.header.frame_id;

    RCLCPP_INFO(
      get_logger(),
      "Received map: %dx%d, resolution = %.3f, frame = %s",
      map_.info.width, map_.info.height,
      map_.info.resolution, frame_id_.c_str());

    if (
      map_.info.width == 0 ||
      map_.info.height == 0 ||
      map_.info.resolution <= 0.0)
    {
      RCLCPP_WARN(get_logger(), "Empty or invalid map.");
      return;
    }

    buildSafeGrid();
    generateKeyPoints();
    publish();
  }

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

  bool segmentLongEnough(const std::vector<int> & cells) const
  {
    if (cells.empty()) {
      return false;
    }

    const double res = map_.info.resolution;
    const double length =
      static_cast<double>(cells.back() - cells.front()) * res;

    return length >= min_segment_length_;
  }

  geometry_msgs::msg::Quaternion quatFromYaw(double yaw) const
  {
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
    return q;
  }

  void generateKeyPoints()
  {
    key_points_.poses.clear();
    segments_.clear();

    const int w = static_cast<int>(map_.info.width);
    const int h = static_cast<int>(map_.info.height);
    const double res = map_.info.resolution;

    int row_step = static_cast<int>(std::round(line_spacing_ / res));
    if (row_step < 1) {
      row_step = 1;
    }

    int max_gap_cells = 1;
    if (max_gap_ > 0.0) {
      max_gap_cells = static_cast<int>(std::ceil(max_gap_ / res));
      if (max_gap_cells < 1) {
        max_gap_cells = 1;
      }
    }

    const int start_row = row_step / 2;

    // true -> едем слева направо
    // false -> едем справа налево
    bool forward = true;

    for (int my = start_row; my < h; my += row_step) {
      std::vector<int> free_x;
      free_x.reserve(w);

      for (int mx = 0; mx < w; ++mx) {
        if (safe_grid_[index(mx, my)]) {
          free_x.push_back(mx);
        }
      }

      if (free_x.empty()) {
        continue;
      }

      // Разбиваем свободные клетки на сегменты.
      std::vector<std::vector<int>> row_segments;
      std::vector<int> current;
      current.push_back(free_x.front());

      for (size_t i = 1; i < free_x.size(); ++i) {
        const int gap = free_x[i] - free_x[i - 1];

        if (gap <= max_gap_cells) {
          current.push_back(free_x[i]);
        } else {
          if (segmentLongEnough(current)) {
            row_segments.push_back(current);
          }

          current.clear();
          current.push_back(free_x[i]);
        }
      }

      if (segmentLongEnough(current)) {
        row_segments.push_back(current);
      }

      if (row_segments.empty()) {
        continue;
      }

      // Если направление назад, обрабатываем сегменты справа налево.
      if (!forward) {
        std::reverse(row_segments.begin(), row_segments.end());
      }

      const double y =
        map_.info.origin.position.y +
        (static_cast<double>(my) + 0.5) * res;

      const double yaw = forward ? 0.0 : M_PI;
      const auto orientation = quatFromYaw(yaw);

      for (const auto & seg_cells : row_segments) {
        if (seg_cells.empty()) {
          continue;
        }

        const int start_cell = forward ? seg_cells.front() : seg_cells.back();
        const int end_cell = forward ? seg_cells.back() : seg_cells.front();

        geometry_msgs::msg::Pose start_pose;
        start_pose.position.x =
          map_.info.origin.position.x +
          (static_cast<double>(start_cell) + 0.5) * res;

        start_pose.position.y = y;
        start_pose.position.z = 0.0;
        start_pose.orientation = orientation;

        geometry_msgs::msg::Pose end_pose;
        end_pose.position.x =
          map_.info.origin.position.x +
          (static_cast<double>(end_cell) + 0.5) * res;

        end_pose.position.y = y;
        end_pose.position.z = 0.0;
        end_pose.orientation = orientation;

        key_points_.poses.push_back(start_pose);
        key_points_.poses.push_back(end_pose);

        segments_.push_back(Segment{start_pose, end_pose});
      }

      // Меняем направление только если строка реально дала сегменты.
      forward = !forward;
    }

    RCLCPP_INFO(
      get_logger(),
      "Generated %zu key points, %zu segments.",
      key_points_.poses.size(), segments_.size());

    if (key_points_.poses.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "No coverage points generated. Check robot_width, clearance_radius and map occupancy.");
    }
  }

  void publish()
  {
    key_points_.header.frame_id = frame_id_;
    key_points_.header.stamp = now();
    key_points_pub_->publish(key_points_);

    publishMarkers();
  }

  void publishMarkers()
  {
    visualization_msgs::msg::MarkerArray markers;

    // Сначала удаляем старые маркеры.
    visualization_msgs::msg::Marker delete_all;
    delete_all.header.frame_id = frame_id_;
    delete_all.header.stamp = now();
    delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(delete_all);

    if (key_points_.poses.empty()) {
      markers_pub_->publish(markers);
      return;
    }

    // ==================== Линия порядка объезда ====================
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

    for (const auto & p : key_points_.poses) {
      line.points.push_back(p.position);
    }

    markers.markers.push_back(line);

    // ==================== Точки ====================
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

    for (const auto & p : key_points_.poses) {
      spheres.points.push_back(p.position);
    }

    markers.markers.push_back(spheres);

    // ==================== Стрелки направлений ====================
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

      arrow.scale.x = 0.02;  // shaft diameter
      arrow.scale.y = 0.06;  // head diameter
      arrow.scale.z = 0.10;  // head length

      arrow.color.r = 0.2;
      arrow.color.g = 0.6;
      arrow.color.b = 1.0;
      arrow.color.a = 0.9;

      markers.markers.push_back(arrow);
    }

    markers_pub_->publish(markers);
  }

  // ==================== Параметры ====================
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

  // ==================== Состояние ====================
  std::string frame_id_;
  nav_msgs::msg::OccupancyGrid map_;
  std::vector<bool> safe_grid_;

  geometry_msgs::msg::PoseArray key_points_;
  std::vector<Segment> segments_;

  // ==================== ROS ====================
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr key_points_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionPlanner>());
  rclcpp::shutdown();
  return 0;
}