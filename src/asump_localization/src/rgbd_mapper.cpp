#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

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

    color_topic_ = declare_parameter<std::string>("color_topic", "/camera/camera/color/image_raw");
    depth_topic_ = declare_parameter<std::string>("depth_topic", "/camera/camera/depth/image_rect_raw");
    info_topic_  = declare_parameter<std::string>("info_topic",  "/camera/camera/color/camera_info");

    map_topic_   = declare_parameter<std::string>("map_topic", "rgbd_map");
    default_save_path_ = declare_parameter<std::string>("default_save_path", "/tmp/map3d.pcd");
    save_binary_       = declare_parameter<bool>("save_binary", true);

    voxel_leaf_size_                = declare_parameter<double>("voxel_leaf_size", 0.05);
    keyframe_translation_threshold_ = declare_parameter<double>("keyframe_translation_threshold", 0.3);
    keyframe_rotation_threshold_    = declare_parameter<double>("keyframe_rotation_threshold", 0.3);
    tf_timeout_                     = declare_parameter<double>("tf_timeout", 0.2);

    min_depth_  = declare_parameter<double>("min_depth",  0.2);
    max_depth_  = declare_parameter<double>("max_depth",  5.0);
    depth_step_ = declare_parameter<int>   ("depth_step", 2);
    queue_size_ = declare_parameter<int>   ("sync_queue_size", 10);

    // ===================== TF =====================
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ===================== Публикация карты =====================
    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      map_topic_, rclcpp::SensorDataQoS());

    // ===================== Подписки на RGB-D + CameraInfo =====================
    color_sub_.subscribe(this, color_topic_, rmw_qos_profile_sensor_data);
    depth_sub_.subscribe(this, depth_topic_, rmw_qos_profile_sensor_data);
    info_sub_ .subscribe(this, info_topic_,  rmw_qos_profile_sensor_data);

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image,
      sensor_msgs::msg::Image,
      sensor_msgs::msg::CameraInfo>;

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(queue_size_), color_sub_, depth_sub_, info_sub_);
    sync_->registerCallback(
      std::bind(&PointCloudMapperNode::rgbdCallback, this,
                std::placeholders::_1,
                std::placeholders::_2,
                std::placeholders::_3));

    // ===================== Глобальная карта =====================
    global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    has_last_keyframe_pose_    = false;
    last_keyframe_position_    = {0.0, 0.0, 0.0};
    last_keyframe_orientation_ = {1.0, 0.0, 0.0, 0.0};

    RCLCPP_INFO(get_logger(),
      "PointCloudMapper (RGB-D) started:\n"
      "  color='%s'\n  depth='%s'\n  info='%s'\n  map='%s'\n"
      "  frames: %s -> %s\n  depth range: [%.2f, %.2f] m, step=%d",
      color_topic_.c_str(), depth_topic_.c_str(), info_topic_.c_str(),
      map_topic_.c_str(), map_frame_.c_str(), base_frame_.c_str(),
      min_depth_, max_depth_, depth_step_);

    // ===================== Сервис сохранения карты =====================
    save_map_srv_ = create_service<std_srvs::srv::Trigger>(
      "save_map3d",
      std::bind(&PointCloudMapperNode::saveMapCallback, this,
                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    RCLCPP_INFO(get_logger(), "Service 'save_map3d' is ready. Default path: %s",
                default_save_path_.c_str());
  }

private:
  // ===================== Основной callback =====================
  void rgbdCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & color_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg)
  {
    // 1) Проверка согласованности размеров
    if (color_msg->width != depth_msg->width ||
        color_msg->height != depth_msg->height)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Разрешения color (%ux%u) и depth (%ux%u) не совпадают.",
        color_msg->width, color_msg->height,
        depth_msg->width, depth_msg->height);
      return;
    }

    // 2) Извлекаем внутренние параметры камеры
    const double fx = info_msg->k[0];
    const double fy = info_msg->k[4];
    const double cx = info_msg->k[2];
    const double cy = info_msg->k[5];
    if (fx <= 0.0 || fy <= 0.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Некорректная матрица камеры (fx=%.2f, fy=%.2f). Пропуск.", fx, fy);
      return;
    }

    // 3) Конвертируем изображения в cv::Mat
    cv::Mat color_bgr;
    cv::Mat depth_float;
    try {
      if (color_msg->encoding == sensor_msgs::image_encodings::BGR8) {
        color_bgr = cv::Mat(color_msg->height, color_msg->width, CV_8UC3,
                            const_cast<uint8_t *>(color_msg->data.data())).clone();
      } else if (color_msg->encoding == sensor_msgs::image_encodings::RGB8) {
        cv::Mat rgb(color_msg->height, color_msg->width, CV_8UC3,
                    const_cast<uint8_t *>(color_msg->data.data()));
        cv::cvtColor(rgb, color_bgr, cv::COLOR_RGB2BGR);
      } else if (color_msg->encoding == sensor_msgs::image_encodings::MONO8) {
        cv::Mat mono(color_msg->height, color_msg->width, CV_8UC1,
                     const_cast<uint8_t *>(color_msg->data.data()));
        cv::cvtColor(mono, color_bgr, cv::COLOR_GRAY2BGR);
      } else {
        auto ptr = cv_bridge::toCvShare(color_msg, sensor_msgs::image_encodings::BGR8);
        color_bgr = ptr->image.clone();
      }

      if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
        cv::Mat d16(depth_msg->height, depth_msg->width, CV_16UC1,
                    const_cast<uint8_t *>(depth_msg->data.data()));
        d16.convertTo(depth_float, CV_32F, 0.001);
      } else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
        depth_float = cv::Mat(depth_msg->height, depth_msg->width, CV_32FC1,
                              const_cast<uint8_t *>(depth_msg->data.data())).clone();
      } else {
        auto ptr = cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
        depth_float = ptr->image.clone();
      }
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "cv_bridge error: %s", e.what());
      return;
    }

    // 4) Строим облако точек в оптическом фрейме камеры
    auto cloud_cam = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    cloud_cam->reserve((color_bgr.rows / depth_step_) * (color_bgr.cols / depth_step_));

    const int H = depth_float.rows;
    const int W = depth_float.cols;
    const int step = std::max(1, depth_step_);

    for (int v = 0; v < H; v += step) {
      const float * d_row = depth_float.ptr<float>(v);
      const cv::Vec3b * c_row = color_bgr.ptr<cv::Vec3b>(v);
      for (int u = 0; u < W; u += step) {
        const float z = d_row[u];
        if (!std::isfinite(z) || z == 0.0f ||
            z < static_cast<float>(min_depth_) ||
            z > static_cast<float>(max_depth_))
        {
          continue;
        }
        const float x = static_cast<float>((u - cx) * z / fx);
        const float y = static_cast<float>((v - cy) * z / fy);

        pcl::PointXYZRGB pt;
        pt.x = x;
        pt.y = y;
        pt.z = z;
        pt.b = c_row[u][0];
        pt.g = c_row[u][1];
        pt.r = c_row[u][2];
        cloud_cam->push_back(pt);
      }
    }

    if (cloud_cam->empty()) {
      return;
    }

    // 5) Получаем TF map -> camera_optical_frame
    geometry_msgs::msg::TransformStamped tf_map_cam;
    try {
      tf_map_cam = tf_buffer_->lookupTransform(
        map_frame_, color_msg->header.frame_id,
        color_msg->header.stamp,
        rclcpp::Duration::from_seconds(tf_timeout_));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "TF %s -> %s недоступен: %s",
        map_frame_.c_str(), color_msg->header.frame_id.c_str(), ex.what());
      return;
    }

    const auto & t = tf_map_cam.transform.translation;
    const auto & r = tf_map_cam.transform.rotation;

    // 6) Проверка ключевого кадра
    if (!isKeyframe(t.x, t.y, t.z, r.w, r.x, r.y, r.z)) {
      return;
    }
    last_keyframe_position_    = {t.x, t.y, t.z};
    last_keyframe_orientation_ = {r.w, r.x, r.y, r.z};
    has_last_keyframe_pose_    = true;

    // 7) Трансформируем облако в map_frame через Eigen::Affine3f
    Eigen::Affine3f T = Eigen::Affine3f::Identity();
    T.translation() << static_cast<float>(t.x),
                       static_cast<float>(t.y),
                       static_cast<float>(t.z);
    T.linear() = Eigen::Quaternionf(
        static_cast<float>(r.w),
        static_cast<float>(r.x),
        static_cast<float>(r.y),
        static_cast<float>(r.z)).toRotationMatrix();

    auto cloud_map = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::transformPointCloud(*cloud_cam, *cloud_map, T);

    // 8) VoxelGrid downsampling
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
    voxel.setInputCloud(cloud_map);
    voxel.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    auto cloud_filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    voxel.filter(*cloud_filtered);

    if (cloud_filtered->empty()) {
      return;
    }

    // 9) Добавляем в глобальную карту
    std::size_t map_size = 0;
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      *global_map_ += *cloud_filtered;
      map_size = global_map_->size();
    }

    publishMap();
    RCLCPP_INFO(get_logger(), "Keyframe added. Points in frame: %zu. Global map: %zu points.",
                cloud_filtered->size(), map_size);
  }

  // ===================== Keyframe logic =====================
  bool isKeyframe(double x, double y, double z,
                  double qw, double qx, double qy, double qz)
  {
    if (!has_last_keyframe_pose_) {
      return true;
    }
    const double dx   = x - last_keyframe_position_[0];
    const double dy   = y - last_keyframe_position_[1];
    const double dz   = z - last_keyframe_position_[2];
    const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

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

  // ===================== Сервис сохранения =====================
  void saveMapCallback(
    const std::shared_ptr<rmw_request_id_t> /*request_header*/,
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
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      pcl::toROSMsg(*global_map_, map_msg);
    }
    map_msg.header.frame_id = map_frame_;
    map_msg.header.stamp = now();
    map_pub_->publish(map_msg);
  }

  // ===================== Члены класса =====================
  std::string map_frame_;
  std::string base_frame_;
  std::string color_topic_;
  std::string depth_topic_;
  std::string info_topic_;
  std::string map_topic_;
  std::string default_save_path_;
  bool save_binary_;

  double voxel_leaf_size_;
  double keyframe_translation_threshold_;
  double keyframe_rotation_threshold_;
  double tf_timeout_;
  double min_depth_;
  double max_depth_;
  int depth_step_;
  int queue_size_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  message_filters::Subscriber<sensor_msgs::msg::Image>       color_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image>       depth_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo>  info_sub_;
  std::shared_ptr<message_filters::Synchronizer<
    message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image,
      sensor_msgs::msg::Image,
      sensor_msgs::msg::CameraInfo>>> sync_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_map_srv_;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_map_;
  bool has_last_keyframe_pose_;
  std::array<double, 3> last_keyframe_position_;
  std::array<double, 4> last_keyframe_orientation_;

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