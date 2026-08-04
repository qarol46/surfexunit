#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl_conversions/pcl_conversions.h"

#include <mutex>
#include <pcl/io/pcd_io.h>
#include "std_srvs/srv/trigger.hpp"

namespace localization
{

class PointCloudMapperNode : public rclcpp::Node
{
public:
  PointCloudMapperNode()
  : Node("pointcloud_mapper")
  {
    // ===================== Параметры =====================
    map_frame_   = declare_parameter<std::string>("map_frame", "map");
    base_frame_  = declare_parameter<std::string>("base_frame", "base_footprint");
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/depth_camera/depth/points");
    map_topic_   = declare_parameter<std::string>("map_topic", "rgbd_map");
    default_save_path_ = declare_parameter<std::string>("default_save_path", "/tmp/map3d.pcd");
    save_binary_       = declare_parameter<bool>("save_binary", true);

    voxel_leaf_size_                = declare_parameter<double>("voxel_leaf_size", 0.2);
    keyframe_translation_threshold_ = declare_parameter<double>("keyframe_translation_threshold", 0.5);
    keyframe_rotation_threshold_    = declare_parameter<double>("keyframe_rotation_threshold", 0.5);
    tf_timeout_                     = declare_parameter<double>("tf_timeout", 0.2);

    // ===================== TF =====================
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ===================== Публикация карты =====================
    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      map_topic_, rclcpp::SensorDataQoS());

    // ===================== Подписка на облако точек =====================
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PointCloudMapperNode::cloudCallback, this, std::placeholders::_1));

    // ===================== Глобальная карта =====================
    global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

    has_last_keyframe_pose_    = false;
    last_keyframe_position_    = {0.0, 0.0, 0.0};
    last_keyframe_orientation_ = {1.0, 0.0, 0.0, 0.0};  // [w, x, y, z]

    RCLCPP_INFO(get_logger(),
      "PointCloudMapper started: cloud='%s' -> map='%s' (frames: %s -> %s)",
      cloud_topic_.c_str(), map_topic_.c_str(), map_frame_.c_str(), base_frame_.c_str());

    // ===================== Сервис сохранения карты =====================
    save_map_srv_ = create_service<std_srvs::srv::Trigger>(
      "save_map3d",
      std::bind(&PointCloudMapperNode::saveMapCallback, this,
                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    RCLCPP_INFO(get_logger(), "Service 'save_map3d' is ready. Default path: %s",
                default_save_path_.c_str());
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
  {
    // 1) Актуальная поза base_frame в map_frame на момент времени облака.
    //    TF-буфер уже содержит коррекцию от slam_toolbox (map -> odom).
    geometry_msgs::msg::TransformStamped tf_map_base;
    try {
      tf_map_base = tf_buffer_->lookupTransform(
        map_frame_, base_frame_, cloud_msg->header.stamp,
        rclcpp::Duration::from_seconds(tf_timeout_));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "TF %s -> %s недоступен: %s", map_frame_.c_str(), base_frame_.c_str(), ex.what());
      return;
    }

    const auto & t = tf_map_base.transform.translation;
    const auto & r = tf_map_base.transform.rotation;

    // 2) Проверка ключевого кадра
    if (!isKeyframe(t.x, t.y, t.z, r.w, r.x, r.y, r.z)) {
      return;
    }

    // 3) Запоминаем позу ключевого кадра
    last_keyframe_position_    = {t.x, t.y, t.z};
    last_keyframe_orientation_ = {r.w, r.x, r.y, r.z};
    has_last_keyframe_pose_    = true;

    // 4) Трансформируем облако в map_frame
    sensor_msgs::msg::PointCloud2 cloud_in_map;
    try {
      tf_buffer_->transform(*cloud_msg, cloud_in_map, map_frame_);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Не удалось трансформировать облако в %s: %s", map_frame_.c_str(), ex.what());
      return;
    }

    // 5) Конвертируем в PCL
    auto cloud_pcl = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::fromROSMsg(cloud_in_map, *cloud_pcl);

    // 6) Удаляем NaN/Inf точки
    auto cloud_clean = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    cloud_clean->reserve(cloud_pcl->size());
    for (const auto & pt : *cloud_pcl) {
      if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
        cloud_clean->push_back(pt);
      }
    }

    // 7) Downsampling через VoxelGrid
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
    voxel.setInputCloud(cloud_clean);
    voxel.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    auto cloud_filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    voxel.filter(*cloud_filtered);

    if (cloud_filtered->empty()) {
      return;
    }

    // 8) Регистрируем в глобальную карту
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      *global_map_ += *cloud_filtered;
    }

    // 9) Публикуем карту
    publishMap();

    RCLCPP_INFO(get_logger(), "Keyframe added. Global map: %zu points.", global_map_->size());
  }

  bool isKeyframe(double x, double y, double z,
                  double qw, double qx, double qy, double qz)
  {
    // Первый кадр всегда ключевой
    if (!has_last_keyframe_pose_) {
      return true;
    }

    // Перемещение
    const double dx   = x - last_keyframe_position_[0];
    const double dy   = y - last_keyframe_position_[1];
    const double dz   = z - last_keyframe_position_[2];
    const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

    // Поворот (угол между кватернионами)
    const double angle = quaternionAngle(
      last_keyframe_orientation_[0], last_keyframe_orientation_[1],
      last_keyframe_orientation_[2], last_keyframe_orientation_[3],
      qw, qx, qy, qz);

    return (dist > keyframe_translation_threshold_) ||
           (angle > keyframe_rotation_threshold_);
  }

  double quaternionAngle(double w1, double x1, double y1, double z1,
                         double w2, double x2, double y2, double z2) const
  {
    double dot = w1 * w2 + x1 * x2 + y1 * y2 + z1 * z2;
    dot = std::min(1.0, std::abs(dot));
    return 2.0 * std::acos(dot);
  }

  void saveMapCallback(const std::shared_ptr<rmw_request_id_t> /*request_header*/,
                       const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_copy;
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      if (global_map_->empty()) {
        response->success = false;
        response->message = "Карта пуста, нечего сохранять.";
        RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
        return;
      }
      // Делаем копию, чтобы не блокировать карту надолго
      map_copy = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(*global_map_);
    }

    int result = -1;
    if (save_binary_) {
      result = pcl::io::savePCDFileBinary(default_save_path_, *map_copy);
    } else {
      result = pcl::io::savePCDFileASCII(default_save_path_, *map_copy);
    }

    if (result == 0) {
      response->success = true;
      response->message = "Карта сохранена: " + default_save_path_ +
                          " (" + std::to_string(map_copy->size()) + " точек)";
      RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
    } else {
      response->success = false;
      response->message = "Ошибка сохранения в " + default_save_path_;
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    }
  }

  void publishMap()
  {
    sensor_msgs::msg::PointCloud2 map_msg;
    pcl::toROSMsg(*global_map_, map_msg);
    map_msg.header.frame_id = map_frame_;
    map_msg.header.stamp = now();
    map_pub_->publish(map_msg);
  }

  // ===================== Члены =====================
  std::string map_frame_;
  std::string base_frame_;
  std::string cloud_topic_;
  std::string map_topic_;

  double voxel_leaf_size_;
  double keyframe_translation_threshold_;
  double keyframe_rotation_threshold_;
  double tf_timeout_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_map_;

  bool has_last_keyframe_pose_;
  std::array<double, 3> last_keyframe_position_;
  std::array<double, 4> last_keyframe_orientation_;  // [w, x, y, z]

  // ===================== Сервис сохранения =====================
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_map_srv_;
  std::string default_save_path_;
  bool save_binary_;
  std::mutex map_mutex_;
};

}  // namespace localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<localization::PointCloudMapperNode>());
  rclcpp::shutdown();
  return 0;
}