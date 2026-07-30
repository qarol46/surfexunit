#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <mutex>
#include <cmath>

class LidarOdometry : public rclcpp::Node
{
public:
  LidarOdometry()
    : Node("lidar_odometry"), has_previous_cloud_(false)
  {
    // Load parameters
    declare_parameter("icp_max_iterations", 50);
    declare_parameter("icp_transformation_epsilon", 1e-8);
    declare_parameter("icp_euclidean_fitness_epsilon", 1e-5);
    declare_parameter("icp_max_correspondence_distance", 0.5);
    declare_parameter("icp_ransac_iterations", 50);
    declare_parameter("voxel_grid_filter_size", 0.05);
    declare_parameter("publish_tf", true);
    declare_parameter("odom_frame_id", "odom");
    declare_parameter("base_frame_id", "base_footprint");
    declare_parameter("lidar_frame_id", "lidar_link");
    declare_parameter("scan_queue_size", 2);
    declare_parameter("debug_publish_cloud", true);
    
    double init_x, init_y, init_theta;
    declare_parameter("initial_pose_x", 0.0);
    declare_parameter("initial_pose_y", 0.0);
    declare_parameter("initial_pose_theta", 0.0);
    
    get_parameter("icp_max_iterations", icp_max_iterations_);
    get_parameter("icp_transformation_epsilon", icp_transformation_epsilon_);
    get_parameter("icp_euclidean_fitness_epsilon", icp_euclidean_fitness_epsilon_);
    get_parameter("icp_max_correspondence_distance", icp_max_correspondence_distance_);
    get_parameter("icp_ransac_iterations", icp_ransac_iterations_);
    get_parameter("voxel_grid_filter_size", voxel_grid_size_);
    get_parameter("publish_tf", publish_tf_);
    get_parameter("odom_frame_id", odom_frame_id_);
    get_parameter("base_frame_id", base_frame_id_);
    get_parameter("lidar_frame_id", lidar_frame_id_);
    get_parameter("scan_queue_size", scan_queue_size_);
    get_parameter("debug_publish_cloud", debug_publish_cloud_);
    get_parameter("initial_pose_x", init_x);
    get_parameter("initial_pose_y", init_y);
    get_parameter("initial_pose_theta", init_theta);
    
    // Initialize pose
    current_pose_ = Eigen::Vector3d(init_x, init_y, init_theta);
    
    // Setup ICP
    setupICP();
    
    // Subscribers and publishers
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", scan_queue_size_, 
      std::bind(&LidarOdometry::scanCallback, this, std::placeholders::_1));
    
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/lidar_odometry", 10);
    
    if (debug_publish_cloud_) {
      cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/aligned_cloud", 10);
      debug_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/debug_cloud", 10);
    }
    
    // TF broadcaster
    if (publish_tf_) {
      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }
    
    RCLCPP_INFO(get_logger(), "Lidar Odometry node initialized with PCL ICP");
    RCLCPP_INFO(get_logger(), "ICP max iterations: %d", icp_max_iterations_);
    RCLCPP_INFO(get_logger(), "ICP max correspondence distance: %.2f m", icp_max_correspondence_distance_);
    RCLCPP_INFO(get_logger(), "Voxel grid size: %.3f m", voxel_grid_size_);
    RCLCPP_INFO(get_logger(), "Initial pose: (%.2f, %.2f, %.2f rad)", 
                init_x, init_y, init_theta);
  }

private:
  void setupICP()
  {
    icp_ = std::make_shared<pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>>();
    
    // Set ICP parameters
    icp_->setMaximumIterations(icp_max_iterations_);
    icp_->setTransformationEpsilon(icp_transformation_epsilon_);
    icp_->setEuclideanFitnessEpsilon(icp_euclidean_fitness_epsilon_);
    icp_->setMaxCorrespondenceDistance(icp_max_correspondence_distance_);
    icp_->setRANSACIterations(icp_ransac_iterations_);
    icp_->setUseReciprocalCorrespondences(false);
    
    RCLCPP_INFO(get_logger(), "PCL ICP configured successfully");
  }
  
  // Convert LaserScan to PCL point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr laserScanToPointCloud(
    const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->reserve(scan->ranges.size());
    
    float angle = scan->angle_min;
    for (size_t i = 0; i < scan->ranges.size(); ++i)
    {
      float range = scan->ranges[i];
      if (range >= scan->range_min && range <= scan->range_max && 
          range > 0.1)  // Ignore too close points
      {
        float x = range * cos(angle);
        float y = range * sin(angle);
        cloud->push_back(pcl::PointXYZ(x, y, 0.0));
      }
      angle += scan->angle_increment;
    }
    
    if (cloud->empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, 
                          "Empty cloud after conversion");
      return nullptr;
    }
    
    // Apply voxel grid filter to downsample
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(voxel_grid_size_, voxel_grid_size_, voxel_grid_size_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    voxel_filter.filter(*filtered_cloud);
    
    if (filtered_cloud->size() < min_reading_points_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, 
                          "Too few points after filtering: %zu", filtered_cloud->size());
      return nullptr;
    }
    
    return filtered_cloud;
  }
  
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto start_time = get_clock()->now();
    
    // Convert current scan to point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud = laserScanToPointCloud(scan_msg);
    
    if (!current_cloud || current_cloud->size() < min_reading_points_) {
      return;
    }
    
    // If no previous cloud, store and return
    if (!has_previous_cloud_)
    {
      previous_cloud_ = current_cloud;
      previous_cloud_time_ = scan_msg->header.stamp;
      has_previous_cloud_ = true;
      RCLCPP_INFO(get_logger(), "First scan received, waiting for next");

      if (debug_publish_cloud_) {
        publishDebugCloud(current_cloud, scan_msg->header.stamp, "base_link");
      }
      return;
    }
    
    try
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>);

      icp_->setInputSource(current_cloud);
      icp_->setInputTarget(previous_cloud_);

      icp_->align(*aligned_cloud);
      
      if (icp_->hasConverged())
      {
        Eigen::Matrix4f transformation = icp_->getFinalTransformation();

        double delta_x = transformation(0, 3);
        double delta_y = transformation(1, 3);
        double delta_theta = std::atan2(transformation(1, 0), transformation(0, 0));
        
        double fitness_score = icp_->getFitnessScore();

        double new_x = current_pose_(0) + delta_x * cos(current_pose_(2)) - delta_y * sin(current_pose_(2));
        double new_y = current_pose_(1) + delta_x * sin(current_pose_(2)) + delta_y * cos(current_pose_(2));
        double new_theta = current_pose_(2) + delta_theta;

        if (new_theta > M_PI) new_theta -= 2 * M_PI;
        if (new_theta < -M_PI) new_theta += 2 * M_PI;
        
        current_pose_ = Eigen::Vector3d(new_x, new_y, new_theta);

        publishOdometry(scan_msg->header.stamp);
        
        if (publish_tf_)
          publishTF(scan_msg->header.stamp);

        if (debug_publish_cloud_) {
          publishAlignedCloud(aligned_cloud, scan_msg->header.stamp);
          publishDebugCloud(current_cloud, scan_msg->header.stamp, "base_link");
        }

        previous_cloud_ = current_cloud;
        previous_cloud_time_ = scan_msg->header.stamp;
        
        auto duration = (get_clock()->now() - start_time).seconds();
        RCLCPP_DEBUG(get_logger(), 
                    "Odometry update: dx=%.3f, dy=%.3f, dtheta=%.3f, pose: (%.3f, %.3f, %.3f), fitness: %.6f, time: %.3f ms",
                    delta_x, delta_y, delta_theta, current_pose_(0), current_pose_(1), current_pose_(2),
                    fitness_score, duration * 1000.0);
      }
      else
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "ICP did not converge");
      }
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(get_logger(), "ICP failed: %s", e.what());
    }
  }
  
  void publishOdometry(const rclcpp::Time& stamp)
  {
    auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();
    odom_msg->header.stamp = stamp;
    odom_msg->header.frame_id = odom_frame_id_;
    odom_msg->child_frame_id = base_frame_id_;

    odom_msg->pose.pose.position.x = current_pose_(0);
    odom_msg->pose.pose.position.y = current_pose_(1);
    odom_msg->pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, current_pose_(2));
    odom_msg->pose.pose.orientation.x = q.x();
    odom_msg->pose.pose.orientation.y = q.y();
    odom_msg->pose.pose.orientation.z = q.z();
    odom_msg->pose.pose.orientation.w = q.w();

    for (int i = 0; i < 36; ++i)
      odom_msg->pose.covariance[i] = 0.0;
    odom_msg->pose.covariance[0] = 0.1;   // x
    odom_msg->pose.covariance[7] = 0.1;   // y
    odom_msg->pose.covariance[35] = 0.1;  // yaw
    
    odom_pub_->publish(std::move(odom_msg));
  }
  
  void publishTF(const rclcpp::Time& stamp)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = odom_frame_id_;
    transform.child_frame_id = base_frame_id_;
    
    transform.transform.translation.x = current_pose_(0);
    transform.transform.translation.y = current_pose_(1);
    transform.transform.translation.z = 0.0;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, current_pose_(2));
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
    
    tf_broadcaster_->sendTransform(transform);
  }
  
  void publishAlignedCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const rclcpp::Time& stamp)
  {
    auto cloud_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*cloud, *cloud_msg);
    cloud_msg->header.stamp = stamp;
    cloud_msg->header.frame_id = odom_frame_id_;
    cloud_pub_->publish(std::move(cloud_msg));
  }
  
  void publishDebugCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const rclcpp::Time& stamp, const std::string& frame_id)
  {
    auto cloud_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*cloud, *cloud_msg);
    cloud_msg->header.stamp = stamp;
    cloud_msg->header.frame_id = frame_id;
    debug_cloud_pub_->publish(std::move(cloud_msg));
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_cloud_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  
  std::mutex mutex_;

  std::shared_ptr<pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>> icp_;
  
  bool has_previous_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr previous_cloud_;
  rclcpp::Time previous_cloud_time_;
  
  Eigen::Vector3d current_pose_;  // x, y, theta

  int icp_max_iterations_;
  double icp_transformation_epsilon_;
  double icp_euclidean_fitness_epsilon_;
  double icp_max_correspondence_distance_;
  int icp_ransac_iterations_;
  double voxel_grid_size_;
  bool publish_tf_;
  bool debug_publish_cloud_;
  std::string odom_frame_id_;
  std::string base_frame_id_;
  std::string lidar_frame_id_;
  int scan_queue_size_;
  const int min_reading_points_ = 10;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LidarOdometry>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}