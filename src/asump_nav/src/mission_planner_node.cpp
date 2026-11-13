// mission_planner_node.cpp
// Coverage planner: dominant wall orientation + maximal rectangle decomposition
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <asump_localization/srv/get_waypoint.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

using std::placeholders::_1;
using std::placeholders::_2;
using Trigger = std_srvs::srv::Trigger;
using GetWaypoint = asump_localization::srv::GetWaypoint;

static constexpr double kPi = 3.14159265358979323846;

class MissionPlanner : public rclcpp::Node
{
public:
  MissionPlanner()
  : Node("mission_planner")
  {
    // ==================== Параметры ====================
    map_topic_ = declare_parameter<std::string>("map_topic", "map");
    global_frame_ = declare_parameter<std::string>("global_frame", "map");
    robot_width_ = declare_parameter<double>("robot_width", 0.76);
    pass_overlap_ = declare_parameter<double>("pass_overlap", 0.0);
    line_spacing_ = declare_parameter<double>("line_spacing", 0.5);
    clearance_radius_ = declare_parameter<double>("clearance_radius", 0.45);
    min_segment_length_ = declare_parameter<double>("min_segment_length", 0.6);
    min_rect_short_side_ = declare_parameter<double>("min_rect_short_side", 0.30);
    occupied_threshold_ = declare_parameter<int>("occupied_threshold", 20);
    treat_unknown_as_occupied_ = declare_parameter<bool>("treat_unknown_as_occupied", true);
    start_service_name_ = declare_parameter<std::string>("start_service_name", "/mission/start");
    waypoint_service_name_ = declare_parameter<std::string>("waypoint_service_name", "/mission/get_waypoint");

    // Ориентация маршрута
    orientation_mode_ = declare_parameter<std::string>("orientation_mode", "auto_wall");
    // auto_wall | fixed | none
    fixed_orientation_ = declare_parameter<double>("fixed_orientation", 0.0);   // рад
    orientation_offset_ = declare_parameter<double>("orientation_offset", 0.0); // рад, добавка к авто
    angle_window_deg_ = declare_parameter<double>("angle_window_deg", 6.0);
    angle_step_deg_ = declare_parameter<double>("angle_step_deg", 1.0);

    // Декомпозиция
    grid_downsample_ = declare_parameter<int>("grid_downsample", 4);
    max_rectangles_ = declare_parameter<int>("max_rectangles", 400);
    force_sweep_axis_ = declare_parameter<std::string>("force_sweep_axis", "auto"); // auto | x | y
    publish_debug_grid_ = declare_parameter<bool>("publish_debug_grid", true);

    // Совместимость со старыми launch-файлами (не используется)
    max_gap_ = declare_parameter<double>("max_gap", 0.5);
    (void)max_gap_;

    if (line_spacing_ <= 0.0) {
      line_spacing_ = robot_width_ * (1.0 - pass_overlap_);
      if (line_spacing_ <= 0.0) line_spacing_ = 0.3;
    }
    if (clearance_radius_ < 0.0) clearance_radius_ = 0.0;
    if (grid_downsample_ < 1) grid_downsample_ = 1;
    if (max_rectangles_ < 1) max_rectangles_ = 1;

    // ==================== Publishers ====================
    rclcpp::QoS pub_qos(1);
    pub_qos.transient_local();
    key_points_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("mission/key_points", pub_qos);
    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("mission/key_points_markers", pub_qos);
    debug_grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("mission/coverage_grid", pub_qos);

    // ==================== Subscriber ====================
    rclcpp::QoS sub_qos(1);
    sub_qos.transient_local();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, sub_qos, std::bind(&MissionPlanner::mapCallback, this, _1));

    // ==================== Services ====================
    start_srv_ = create_service<Trigger>(
      start_service_name_, std::bind(&MissionPlanner::startServiceCallback, this, _1, _2));
    waypoint_srv_ = create_service<GetWaypoint>(
      waypoint_service_name_, std::bind(&MissionPlanner::waypointServiceCallback, this, _1, _2));

    RCLCPP_INFO(get_logger(), "Mission planner started (maximal rectangles + wall orientation).");
    RCLCPP_INFO(get_logger(), "Map topic: %s", map_topic_.c_str());
    RCLCPP_INFO(get_logger(), "line_spacing=%.3f m, clearance_radius=%.3f m, downsample=%d",
                line_spacing_, clearance_radius_, grid_downsample_);
    RCLCPP_INFO(get_logger(), "Orientation mode: %s (offset %.2f deg)",
                orientation_mode_.c_str(), orientation_offset_ * 180.0 / kPi);
  }

private:
  // ==================== Types ====================
  struct Rect { int x = 0, y = 0, w = 0, h = 0; };
  struct Segment { geometry_msgs::msg::Pose start, end; };
  struct WorkGrid
  {
    int w = 0, h = 0;
    double res = 0.05;
    double ox = 0.0, oy = 0.0;      // угол клетки (0,0) в повёрнутой системе
    std::vector<uint8_t> free;      // 1 = свободно и безопасно
    size_t idx(int x, int y) const { return static_cast<size_t>(y) * w + x; }
  };
  using PoseVec = std::vector<geometry_msgs::msg::Pose>;

  // ==================== Map ====================
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    bool first = !has_map_;
    map_ = *msg;
    has_map_ = true;
    frame_id_ = map_.header.frame_id.empty() ? global_frame_ : map_.header.frame_id;
    map_yaw_ = yawFromQuat(map_.info.origin.orientation);
    if (first) {
      RCLCPP_INFO(get_logger(), "Map received: %ux%u, res=%.3f, frame=%s, origin_yaw=%.2f deg",
                  map_.info.width, map_.info.height, map_.info.resolution,
                  frame_id_.c_str(), map_yaw_ * 180.0 / kPi);
    }
  }

  // ==================== Math helpers ====================
  static double yawFromQuat(const geometry_msgs::msg::Quaternion & q)
  {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  static geometry_msgs::msg::Quaternion quatFromYaw(double yaw)
  {
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0; q.y = 0.0;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
    return q;
  }

  static double foldPi(double a)  // -> [0, pi)
  {
    while (a < 0.0) a += kPi;
    while (a >= kPi) a -= kPi;
    return a;
  }

  // local (метры в системе карты) -> world
  geometry_msgs::msg::Point mapLocalToWorld(double lx, double ly) const
  {
    const double c = std::cos(map_yaw_), s = std::sin(map_yaw_);
    geometry_msgs::msg::Point p;
    p.x = map_.info.origin.position.x + lx * c - ly * s;
    p.y = map_.info.origin.position.y + lx * s + ly * c;
    p.z = 0.0;
    return p;
  }

  // world -> local (метры)
  void worldToMapLocal(double wx, double wy, double & lx, double & ly) const
  {
    const double c = std::cos(map_yaw_), s = std::sin(map_yaw_);
    const double dx = wx - map_.info.origin.position.x;
    const double dy = wy - map_.info.origin.position.y;
    lx = dx * c + dy * s;
    ly = -dx * s + dy * c;
  }

  // q (повёрнутая система) -> world
  geometry_msgs::msg::Point qToWorld(double qx, double qy, double theta) const
  {
    const double c = std::cos(theta), s = std::sin(theta);
    geometry_msgs::msg::Point p;
    p.x = center_x_ + qx * c - qy * s;
    p.y = center_y_ + qx * s + qy * c;
    p.z = 0.0;
    return p;
  }

  // ==================== Original grid helpers ====================
  size_t mapIdx(int x, int y) const
  {
    return static_cast<size_t>(y) * map_.info.width + x;
  }

  bool inMap(int x, int y) const
  {
    return x >= 0 && y >= 0 &&
           x < static_cast<int>(map_.info.width) &&
           y < static_cast<int>(map_.info.height);
  }

  void computeBlockedAndSafe()
  {
    const int w = static_cast<int>(map_.info.width);
    const int h = static_cast<int>(map_.info.height);
    const size_t n = static_cast<size_t>(w) * h;
    blocked_.assign(n, 0);

    size_t free_count = 0;
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        const int8_t cost = map_.data[mapIdx(x, y)];
        bool blocked = (cost < 0) ? treat_unknown_as_occupied_
                                  : (cost >= occupied_threshold_);
        blocked_[mapIdx(x, y)] = blocked ? 1 : 0;
        if (!blocked) ++free_count;
      }
    }

    // Chamfer distance transform (расстояние до ближайшего препятствия в клетках).
    // Вне карты считаем препятствием.
    const double INF = 1e9;
    const double d1 = 1.0, d2 = std::sqrt(2.0);
    std::vector<double> dist(n, INF);
    for (size_t i = 0; i < n; ++i) {
      if (blocked_[i]) dist[i] = 0.0;
    }

    auto D = [&](int x, int y) -> double {
      if (!inMap(x, y)) return 0.0;                 // за границей = препятствие
      return dist[mapIdx(x, y)];
    };

    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        double & d = dist[mapIdx(x, y)];
        if (d == 0.0) continue;
        d = std::min(d, D(x - 1, y) + d1);
        d = std::min(d, D(x, y - 1) + d1);
        d = std::min(d, D(x - 1, y - 1) + d2);
        d = std::min(d, D(x + 1, y - 1) + d2);
      }
    }
    for (int y = h - 1; y >= 0; --y) {
      for (int x = w - 1; x >= 0; --x) {
        double & d = dist[mapIdx(x, y)];
        if (d == 0.0) continue;
        d = std::min(d, D(x + 1, y) + d1);
        d = std::min(d, D(x, y + 1) + d1);
        d = std::min(d, D(x + 1, y + 1) + d2);
        d = std::min(d, D(x - 1, y + 1) + d2);
      }
    }

    safe_.assign(n, 0);
    size_t safe_count = 0;
    const double need_cells = clearance_radius_ / map_.info.resolution;
    for (size_t i = 0; i < n; ++i) {
      if (blocked_[i]) continue;
      // -0.5 клетки — консервативно (граница клетки, а не центр)
      if (dist[i] - 0.5 >= need_cells) {
        safe_[i] = 1;
        ++safe_count;
      }
    }

    RCLCPP_INFO(get_logger(), "Grid: free=%zu cells, safe=%zu cells (clearance %.2f m)",
                free_count, safe_count, clearance_radius_);
    if (safe_count == 0) {
      RCLCPP_ERROR(get_logger(),
        "No safe cells! Уменьши clearance_radius (сейчас %.2f м) или occupied_threshold. "
        "Если подписан на уже раздутую карту (map_inflated), clearance должен быть ~0.05-0.15 м.",
        clearance_radius_);
    }
  }

  bool safeAtWorld(double wx, double wy) const
  {
    double lx, ly;
    worldToMapLocal(wx, wy, lx, ly);
    const int cx = static_cast<int>(std::floor(lx / map_.info.resolution));
    const int cy = static_cast<int>(std::floor(ly / map_.info.resolution));
    if (!inMap(cx, cy)) return false;
    return safe_[mapIdx(cx, cy)] != 0;
  }

  // ==================== Dominant wall orientation ====================
  double dominantWallAngle() const
  {
    const int w = static_cast<int>(map_.info.width);
    const int h = static_cast<int>(map_.info.height);
    const int NB = 180;                       // 1 градус на бин
    std::vector<double> hist(NB, 0.0);

    auto B = [&](int x, int y) -> double {
      if (!inMap(x, y)) return 1.0;
      return blocked_[mapIdx(x, y)] ? 1.0 : 0.0;
    };

    for (int y = 1; y + 1 < h; ++y) {
      for (int x = 1; x + 1 < w; ++x) {
        const double gx =
          (B(x + 1, y - 1) + 2.0 * B(x + 1, y) + B(x + 1, y + 1)) -
          (B(x - 1, y - 1) + 2.0 * B(x - 1, y) + B(x - 1, y + 1));
        const double gy =
          (B(x - 1, y + 1) + 2.0 * B(x, y + 1) + B(x + 1, y + 1)) -
          (B(x - 1, y - 1) + 2.0 * B(x, y - 1) + B(x + 1, y - 1));
        const double mag = std::hypot(gx, gy);
        if (mag < 1.0) continue;
        // нормаль к стене -> касательная (вдоль стены)
        const double tangent = foldPi(std::atan2(gy, gx) + kPi / 2.0);
        const int bin = std::min(NB - 1, static_cast<int>(tangent / kPi * NB));
        hist[bin] += mag;
      }
    }

    // циклическое сглаживание
    for (int pass = 0; pass < 3; ++pass) {
      std::vector<double> tmp(NB, 0.0);
      for (int i = 0; i < NB; ++i) {
        const int a = (i + NB - 2) % NB, b = (i + NB - 1) % NB;
        const int c = (i + 1) % NB, d = (i + 2) % NB;
        tmp[i] = (1.0 * hist[a] + 2.0 * hist[b] + 3.0 * hist[i] +
                  2.0 * hist[c] + 1.0 * hist[d]) / 9.0;
      }
      hist = tmp;
    }

    int best = 0;
    for (int i = 1; i < NB; ++i) {
      if (hist[i] > hist[best]) best = i;
    }
    const double angle = (best + 0.5) * kPi / NB;
    RCLCPP_INFO(get_logger(), "Dominant wall direction (histogram): %.2f deg", angle * 180.0 / kPi);
    return angle;
  }

  // ==================== Working (rotated) grid ====================
  void buildWorkingGrid(double theta, WorkGrid & g) const
  {
    const int mw = static_cast<int>(map_.info.width);
    const int mh = static_cast<int>(map_.info.height);
    const double mres = map_.info.resolution;
    const double c = std::cos(theta), s = std::sin(theta);

    // границы карты в повёрнутой системе
    double minx = std::numeric_limits<double>::max();
    double miny = std::numeric_limits<double>::max();
    double maxx = std::numeric_limits<double>::lowest();
    double maxy = std::numeric_limits<double>::lowest();
    const double lx[4] = {0.0, mw * mres, 0.0, mw * mres};
    const double ly[4] = {0.0, 0.0, mh * mres, mh * mres};
    for (int i = 0; i < 4; ++i) {
      const auto p = mapLocalToWorld(lx[i], ly[i]);
      const double dx = p.x - center_x_, dy = p.y - center_y_;
      const double qx = dx * c + dy * s;
      const double qy = -dx * s + dy * c;
      minx = std::min(minx, qx); maxx = std::max(maxx, qx);
      miny = std::min(miny, qy); maxy = std::max(maxy, qy);
    }

    const int k = grid_downsample_;
    g.res = mres * k;
    g.ox = minx - g.res;                       // +1 клетка паддинга
    g.oy = miny - g.res;
    g.w = static_cast<int>(std::ceil((maxx - minx) / g.res)) + 2;
    g.h = static_cast<int>(std::ceil((maxy - miny) / g.res)) + 2;
    g.free.assign(static_cast<size_t>(g.w) * g.h, 0);

    // консервативная выборка: клетка свободна, только если все k*k подвыборок безопасны
    for (int j = 0; j < g.h; ++j) {
      for (int i = 0; i < g.w; ++i) {
        bool ok = true;
        for (int sy = 0; sy < k && ok; ++sy) {
          const double qy = g.oy + (j + (sy + 0.5) / k) * g.res;
          for (int sx = 0; sx < k && ok; ++sx) {
            const double qx = g.ox + (i + (sx + 0.5) / k) * g.res;
            const auto p = qToWorld(qx, qy, theta);
            if (!safeAtWorld(p.x, p.y)) ok = false;
          }
        }
        g.free[g.idx(i, j)] = ok ? 1 : 0;
      }
    }
  }

  // оценка ориентации: чем длиннее прямые прогоны свободного места, тем лучше
  double orientationScore(double theta) const
  {
    WorkGrid g;
    buildWorkingGrid(theta, g);
    double score = 0.0;
    // горизонтальные прогоны
    for (int j = 0; j < g.h; ++j) {
      int run = 0;
      for (int i = 0; i < g.w; ++i) {
        if (g.free[g.idx(i, j)]) { ++run; }
        else { score += static_cast<double>(run) * run; run = 0; }
      }
      score += static_cast<double>(run) * run;
    }
    // вертикальные прогоны
    for (int i = 0; i < g.w; ++i) {
      int run = 0;
      for (int j = 0; j < g.h; ++j) {
        if (g.free[g.idx(i, j)]) { ++run; }
        else { score += static_cast<double>(run) * run; run = 0; }
      }
      score += static_cast<double>(run) * run;
    }
    return score;
  }

  double determineOrientation() const
  {
    if (orientation_mode_ == "none") {
      RCLCPP_INFO(get_logger(), "Orientation: none (0 deg)");
      return 0.0;
    }
    if (orientation_mode_ == "fixed") {
      const double a = foldPi(fixed_orientation_ + orientation_offset_);
      RCLCPP_INFO(get_logger(), "Orientation: fixed %.2f deg", a * 180.0 / kPi);
      return a;
    }

    const double base = dominantWallAngle();
    const double win = angle_window_deg_ * kPi / 180.0;
    const double step = std::max(0.1, angle_step_deg_) * kPi / 180.0;

    double best_angle = base;
    double best_score = -1.0;
    for (double d = -win; d <= win + 1e-9; d += step) {
      const double a = base + d;
      const double sc = orientationScore(a);
      if (sc > best_score) { best_score = sc; best_angle = a; }
    }
    best_angle = foldPi(best_angle + orientation_offset_);
    RCLCPP_INFO(get_logger(), "Orientation refined: %.2f deg (score %.0f)",
                best_angle * 180.0 / kPi, best_score);
    return best_angle;
  }

  // ==================== Maximal rectangle decomposition ====================
  std::vector<Rect> decomposeMaximalRectangles(const WorkGrid & g) const
  {
    std::vector<Rect> rects;
    const size_t n = static_cast<size_t>(g.w) * g.h;
    std::vector<uint8_t> covered(n, 0);
    std::vector<int> height(g.w + 1, 0);
    std::vector<int> st;
    st.reserve(g.w + 1);

    for (int iter = 0; iter < max_rectangles_; ++iter) {
      std::fill(height.begin(), height.end(), 0);
      int best_area = 0;
      Rect best;

      for (int j = 0; j < g.h; ++j) {
        for (int i = 0; i < g.w; ++i) {
          const size_t id = g.idx(i, j);
          const bool ok = g.free[id] && !covered[id];
          height[i] = ok ? height[i] + 1 : 0;
        }
        height[g.w] = 0;
        st.clear();
        for (int i = 0; i <= g.w; ++i) {
          while (!st.empty() && height[st.back()] >= height[i]) {
            const int top = st.back();
            st.pop_back();
            const int left = st.empty() ? 0 : st.back() + 1;
            const int width = i - left;
            const int area = height[top] * width;
            if (area > best_area) {
              best_area = area;
              best.x = left;
              best.y = j - height[top] + 1;
              best.w = width;
              best.h = height[top];
            }
          }
          st.push_back(i);
        }
      }

      if (best_area <= 0) break;
      for (int yy = best.y; yy < best.y + best.h; ++yy) {
        for (int xx = best.x; xx < best.x + best.w; ++xx) {
          covered[g.idx(xx, yy)] = 1;
        }
      }
      rects.push_back(best);
    }

    RCLCPP_INFO(get_logger(), "Maximal rectangle decomposition: %zu rectangles (grid %dx%d, cell %.3f m)",
                rects.size(), g.w, g.h, g.res);
    return rects;
  }

  // ==================== Ordering ====================
  static double rectCX(const Rect & r) { return r.x + 0.5 * r.w; }
  static double rectCY(const Rect & r) { return r.y + 0.5 * r.h; }

  std::vector<Rect> orderRectangles(std::vector<Rect> rects) const
  {
    std::vector<Rect> ordered;
    if (rects.empty()) return ordered;

    std::sort(rects.begin(), rects.end(), [](const Rect & a, const Rect & b) {
      const double ay = rectCY(a), by = rectCY(b);
      if (std::fabs(ay - by) > 1e-6) return ay < by;
      return rectCX(a) < rectCX(b);
    });

    ordered.push_back(rects.front());
    rects.erase(rects.begin());
    while (!rects.empty()) {
      const Rect & cur = ordered.back();
      double best = std::numeric_limits<double>::max();
      size_t bi = 0;
      for (size_t i = 0; i < rects.size(); ++i) {
        const double dx = rectCX(cur) - rectCX(rects[i]);
        const double dy = rectCY(cur) - rectCY(rects[i]);
        const double d = dx * dx + dy * dy;
        if (d < best) { best = d; bi = i; }
      }
      ordered.push_back(rects[bi]);
      rects.erase(rects.begin() + bi);
    }
    return ordered;
  }

  // ==================== Coverage (в повёрнутой системе q) ====================
  std::vector<double> samplePositions(double lo, double hi, double spacing) const
  {
    std::vector<double> out;
    const double len = hi - lo;
    if (len <= spacing * 0.5) {
      out.push_back(0.5 * (lo + hi));
      return out;
    }
    double p = lo + spacing * 0.5;
    while (p <= hi - spacing * 0.5 + 1e-3) {
      out.push_back(p);
      p += spacing;
    }
    if (out.empty()) out.push_back(0.5 * (lo + hi));
    return out;
  }

  PoseVec buildSnake(const Rect & r, const WorkGrid & g, bool mirror_cross) const
  {
    PoseVec seq;
    const double x_min = g.ox + (r.x + 0.5) * g.res;
    const double x_max = g.ox + (r.x + r.w - 1 + 0.5) * g.res;
    const double y_min = g.oy + (r.y + 0.5) * g.res;
    const double y_max = g.oy + (r.y + r.h - 1 + 0.5) * g.res;
    const double width_m = r.w * g.res;
    const double height_m = r.h * g.res;

    bool along_x;
    if (force_sweep_axis_ == "x") along_x = true;
    else if (force_sweep_axis_ == "y") along_x = false;
    else along_x = (width_m >= height_m);

    std::vector<double> cross = along_x ? samplePositions(y_min, y_max, line_spacing_)
                                        : samplePositions(x_min, x_max, line_spacing_);
    if (mirror_cross) std::reverse(cross.begin(), cross.end());

    bool forward = true;
    for (double cv : cross) {
      geometry_msgs::msg::Pose a, b;
      if (along_x) {
        a.position.y = cv; b.position.y = cv;
        a.position.x = forward ? x_min : x_max;
        b.position.x = forward ? x_max : x_min;
      } else {
        a.position.x = cv; b.position.x = cv;
        a.position.y = forward ? y_min : y_max;
        b.position.y = forward ? y_max : y_min;
      }
      const double yaw = std::atan2(b.position.y - a.position.y,
                                    b.position.x - a.position.x);
      a.orientation = quatFromYaw(yaw);
      b.orientation = quatFromYaw(yaw);
      forward = !forward;

      const double len = std::hypot(b.position.x - a.position.x, b.position.y - a.position.y);
      if (len < min_segment_length_) continue;
      seq.push_back(a);
      seq.push_back(b);
    }
    return seq;
  }

  static PoseVec reversed(const PoseVec & in)
  {
    PoseVec out(in.rbegin(), in.rend());
    for (auto & p : out) {
      p.orientation = quatFromYaw(yawFromQuat(p.orientation) + kPi);
    }
    return out;
  }

  void generateCoverage(const std::vector<Rect> & rects, const WorkGrid & g, double theta)
  {
    waypoints_.clear();
    segments_.clear();
    rect_outlines_.clear();

    size_t skipped = 0;
    bool have_last = false;
    geometry_msgs::msg::Point last_q;

    for (const auto & r : rects) {
      const double short_side = std::min(r.w, r.h) * g.res;
      const double long_side = std::max(r.w, r.h) * g.res;
      if (short_side < min_rect_short_side_ || long_side < min_segment_length_) {
        ++skipped;
        continue;
      }

      // контур прямоугольника (для rviz)
      std::vector<geometry_msgs::msg::Point> outline;
      const double x0 = g.ox + r.x * g.res, x1 = g.ox + (r.x + r.w) * g.res;
      const double y0 = g.oy + r.y * g.res, y1 = g.oy + (r.y + r.h) * g.res;
      outline.push_back(qToWorld(x0, y0, theta));
      outline.push_back(qToWorld(x1, y0, theta));
      outline.push_back(qToWorld(x1, y1, theta));
      outline.push_back(qToWorld(x0, y1, theta));
      rect_outlines_.push_back(outline);

      PoseVec A = buildSnake(r, g, false);
      PoseVec C = buildSnake(r, g, true);
      if (A.empty() && C.empty()) { ++skipped; continue; }

      std::vector<PoseVec> candidates;
      if (!A.empty()) { candidates.push_back(A); candidates.push_back(reversed(A)); }
      if (!C.empty()) { candidates.push_back(C); candidates.push_back(reversed(C)); }

      size_t best = 0;
      if (have_last && candidates.size() > 1) {
        double bd = std::numeric_limits<double>::max();
        for (size_t i = 0; i < candidates.size(); ++i) {
          const double dx = candidates[i].front().position.x - last_q.x;
          const double dy = candidates[i].front().position.y - last_q.y;
          const double d = dx * dx + dy * dy;
          if (d < bd) { bd = d; best = i; }
        }
      }

      const PoseVec & seq = candidates[best];
      for (size_t i = 0; i + 1 < seq.size(); i += 2) {
        Segment s;
        s.start = seq[i];
        s.end = seq[i + 1];
        segments_q_.push_back(s);
      }
      for (const auto & p : seq) waypoints_q_.push_back(p);
      last_q = seq.back().position;
      have_last = true;
    }

    // q -> world
    for (const auto & p : waypoints_q_) {
      geometry_msgs::msg::Pose w;
      w.position = qToWorld(p.position.x, p.position.y, theta);
      w.orientation = quatFromYaw(yawFromQuat(p.orientation) + theta);
      waypoints_.push_back(w);
    }
    for (const auto & s : segments_q_) {
      Segment w;
      w.start.position = qToWorld(s.start.position.x, s.start.position.y, theta);
      w.start.orientation = quatFromYaw(yawFromQuat(s.start.orientation) + theta);
      w.end.position = qToWorld(s.end.position.x, s.end.position.y, theta);
      w.end.orientation = quatFromYaw(yawFromQuat(s.end.orientation) + theta);
      segments_.push_back(w);
    }

    RCLCPP_INFO(get_logger(), "Coverage: %zu waypoints, %zu segments, %zu rects skipped (too small)",
                waypoints_.size(), segments_.size(), skipped);
    if (waypoints_.empty()) {
      RCLCPP_WARN(get_logger(),
        "Пустой маршрут: проверь clearance_radius, min_rect_short_side, min_segment_length.");
    }
  }

  // ==================== Service: start ====================
  void startServiceCallback(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const auto t0 = std::chrono::steady_clock::now();

    if (!has_map_) {
      response->success = false;
      response->message = "No map received yet";
      return;
    }
    if (map_.info.width == 0 || map_.info.height == 0 || map_.info.resolution <= 0.0 ||
        map_.data.size() != static_cast<size_t>(map_.info.width) * map_.info.height)
    {
      response->success = false;
      response->message = "Empty or invalid map";
      return;
    }

    waypoints_q_.clear();
    segments_q_.clear();

    computeBlockedAndSafe();

    center_x_ = map_.info.origin.position.x;
    center_y_ = map_.info.origin.position.y;
    {
      const auto c = mapLocalToWorld(map_.info.width * map_.info.resolution * 0.5,
                                     map_.info.height * map_.info.resolution * 0.5);
      center_x_ = c.x;
      center_y_ = c.y;
    }

    const double theta = determineOrientation();
    buildWorkingGrid(theta, work_);

    auto rects = decomposeMaximalRectangles(work_);
    const size_t rect_count = rects.size();
    auto ordered = orderRectangles(rects);
    generateCoverage(ordered, work_, theta);

    theta_ = theta;
    current_index_ = -1;
    publishVisualization(theta);

    const double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    response->success = !waypoints_.empty();
    response->message =
      "waypoints=" + std::to_string(waypoints_.size()) +
      ", rects=" + std::to_string(rect_count) +
      ", angle=" + std::to_string(theta * 180.0 / kPi) + " deg" +
      ", mode=" + orientation_mode_ +
      ", time=" + std::to_string(dt) + " s";
    RCLCPP_INFO(get_logger(), "Mission generated in %.2f s: %s", dt, response->message.c_str());
  }

  // ==================== Service: get_waypoint ====================
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
      return;
    }

    if (request->advance) {
      if (current_index_ + 1 < static_cast<int>(waypoints_.size())) {
        ++current_index_;
      } else {
        response->finished = true;
        response->index = current_index_;
        return;
      }
    } else if (current_index_ < 0) {
      current_index_ = 0;
    }

    if (current_index_ < 0 || current_index_ >= static_cast<int>(waypoints_.size())) {
      response->finished = true;
      return;
    }

    response->success = true;
    response->index = current_index_;
    response->total = static_cast<int>(waypoints_.size());
    response->pose.header.frame_id = frame_id_;
    response->pose.header.stamp = now();
    response->pose.pose = waypoints_[current_index_];

    RCLCPP_INFO(get_logger(), "Waypoint %d/%d: (%.2f, %.2f)",
                current_index_ + 1, static_cast<int>(waypoints_.size()),
                response->pose.pose.position.x, response->pose.pose.position.y);
  }

  // ==================== Visualization ====================
  void publishVisualization(double theta)
  {
    key_points_.poses = waypoints_;
    key_points_.header.frame_id = frame_id_;
    key_points_.header.stamp = now();
    key_points_pub_->publish(key_points_);

    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker del;
    del.header.frame_id = frame_id_;
    del.header.stamp = now();
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(del);

    // контуры прямоугольников
    int id = 100;
    for (const auto & outline : rect_outlines_) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = frame_id_;
      m.header.stamp = now();
      m.ns = "coverage_rects";
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::LINE_STRIP;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.orientation.w = 1.0;
      m.scale.x = 0.015;
      m.color.r = 1.0; m.color.g = 0.6; m.color.b = 0.0; m.color.a = 0.8;
      m.points = outline;
      if (!outline.empty()) m.points.push_back(outline.front());
      markers.markers.push_back(m);
    }

    if (!waypoints_.empty()) {
      visualization_msgs::msg::Marker line;
      line.header.frame_id = frame_id_;
      line.header.stamp = now();
      line.ns = "coverage_order"; line.id = 1;
      line.type = visualization_msgs::msg::Marker::LINE_STRIP;
      line.action = visualization_msgs::msg::Marker::ADD;
      line.pose.orientation.w = 1.0;
      line.scale.x = 0.02;
      line.color.g = 1.0; line.color.a = 1.0;
      for (const auto & p : waypoints_) line.points.push_back(p.position);
      markers.markers.push_back(line);

      visualization_msgs::msg::Marker pts;
      pts.header.frame_id = frame_id_;
      pts.header.stamp = now();
      pts.ns = "coverage_points"; pts.id = 2;
      pts.type = visualization_msgs::msg::Marker::SPHERE_LIST;
      pts.action = visualization_msgs::msg::Marker::ADD;
      pts.pose.orientation.w = 1.0;
      pts.scale.x = pts.scale.y = pts.scale.z = 0.07;
      pts.color.r = 1.0; pts.color.g = 0.2; pts.color.b = 0.2; pts.color.a = 1.0;
      for (const auto & p : waypoints_) pts.points.push_back(p.position);
      markers.markers.push_back(pts);

      int sid = 10000;
      for (const auto & s : segments_) {
        visualization_msgs::msg::Marker a;
        a.header.frame_id = frame_id_;
        a.header.stamp = now();
        a.ns = "coverage_segments"; a.id = sid++;
        a.type = visualization_msgs::msg::Marker::ARROW;
        a.action = visualization_msgs::msg::Marker::ADD;
        a.pose.orientation.w = 1.0;
        a.points.push_back(s.start.position);
        a.points.push_back(s.end.position);
        a.scale.x = 0.02; a.scale.y = 0.05; a.scale.z = 0.08;
        a.color.r = 0.2; a.color.g = 0.6; a.color.b = 1.0; a.color.a = 0.9;
        markers.markers.push_back(a);
      }
    }

    // жёлтая стрелка — направление проходов
    {
      visualization_msgs::msg::Marker arrow;
      arrow.header.frame_id = frame_id_;
      arrow.header.stamp = now();
      arrow.ns = "sweep_direction"; arrow.id = 99999;
      arrow.type = visualization_msgs::msg::Marker::ARROW;
      arrow.action = visualization_msgs::msg::Marker::ADD;
      arrow.pose.orientation.w = 1.0;
      geometry_msgs::msg::Point a, b;
      a.x = center_x_; a.y = center_y_; a.z = 0.15;
      b.x = center_x_ + 2.0 * std::cos(theta);
      b.y = center_y_ + 2.0 * std::sin(theta);
      b.z = 0.15;
      arrow.points.push_back(a);
      arrow.points.push_back(b);
      arrow.scale.x = 0.08; arrow.scale.y = 0.16; arrow.scale.z = 0.25;
      arrow.color.r = 1.0; arrow.color.g = 1.0; arrow.color.a = 1.0;
      markers.markers.push_back(arrow);
    }
    markers_pub_->publish(markers);

    if (publish_debug_grid_) {
      nav_msgs::msg::OccupancyGrid dbg;
      dbg.header.frame_id = frame_id_;
      dbg.header.stamp = now();
      dbg.info.resolution = work_.res;
      dbg.info.width = work_.w;
      dbg.info.height = work_.h;
      const auto o = qToWorld(work_.ox, work_.oy, theta);
      dbg.info.origin.position = o;
      dbg.info.origin.orientation = quatFromYaw(theta);
      dbg.data.resize(static_cast<size_t>(work_.w) * work_.h);
      for (size_t i = 0; i < dbg.data.size(); ++i) {
        dbg.data[i] = work_.free[i] ? 0 : 100;
      }
      debug_grid_pub_->publish(dbg);
    }
  }

  // ==================== Members ====================
  // params
  std::string map_topic_, global_frame_, start_service_name_, waypoint_service_name_;
  std::string orientation_mode_, force_sweep_axis_;
  double robot_width_, pass_overlap_, line_spacing_, clearance_radius_;
  double min_segment_length_, min_rect_short_side_, max_gap_;
  double fixed_orientation_, orientation_offset_, angle_window_deg_, angle_step_deg_;
  int occupied_threshold_, grid_downsample_, max_rectangles_;
  bool treat_unknown_as_occupied_, publish_debug_grid_;

  // state
  std::mutex state_mutex_;
  bool has_map_ = false;
  std::string frame_id_;
  nav_msgs::msg::OccupancyGrid map_;
  double map_yaw_ = 0.0;
  std::vector<uint8_t> blocked_, safe_;
  WorkGrid work_;
  double theta_ = 0.0, center_x_ = 0.0, center_y_ = 0.0;
  PoseVec waypoints_q_, waypoints_;
  std::vector<Segment> segments_q_, segments_;
  std::vector<std::vector<geometry_msgs::msg::Point>> rect_outlines_;
  geometry_msgs::msg::PoseArray key_points_;
  int current_index_ = -1;

  // ROS
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr key_points_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr debug_grid_pub_;
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