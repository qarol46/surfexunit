#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Dense>
#include <mutex>
#include <memory>
#include <string>
#include <cmath>
#include <limits>

// Подключаем ваш сервис из пакета asump_localization
#include "asump_localization/srv/initialize_pose.hpp"

class ScanMatcherService : public rclcpp::Node
{
public:
    ScanMatcherService() : Node("scan_matcher_service")
    {
        RCLCPP_INFO(get_logger(), "=== 2D SCAN MATCHER SERVICE STARTING ===");
        
        // Parameters
        this->declare_parameter("map_topic", "/map_base");
        this->declare_parameter("scan_topic", "/scan");
        this->declare_parameter("charger_pose_topic", "/zones/charger_pose");
        this->declare_parameter("output_pose_topic", "/initialpose");
        this->declare_parameter("voxel_size", 0.05);
        this->declare_parameter("map_occupied_threshold", 65);
        this->declare_parameter("max_icp_iterations", 30);
        this->declare_parameter("icp_max_correspondence_distance", 0.5);
        
        // Новые параметры для решения проблемы с Yaw
        this->declare_parameter("laser_yaw_offset", 0.0);          // Физический поворот лидара (рад)
        this->declare_parameter("yaw_search_range_deg", 120.0);     // Радиус поиска угла (град)
        this->declare_parameter("yaw_search_step_deg", 5.0);       // Шаг поиска угла (град)
        this->declare_parameter("coarse_icp_distance", 2.0);       // Радиус для грубого ICP
        
        map_topic_ = this->get_parameter("map_topic").as_string();
        scan_topic_ = this->get_parameter("scan_topic").as_string();
        charger_pose_topic_ = this->get_parameter("charger_pose_topic").as_string();
        output_pose_topic_ = this->get_parameter("output_pose_topic").as_string();
        voxel_size_ = this->get_parameter("voxel_size").as_double();
        map_occupied_threshold_ = this->get_parameter("map_occupied_threshold").as_int();
        max_icp_iterations_ = this->get_parameter("max_icp_iterations").as_int();
        icp_max_correspondence_distance_ = this->get_parameter("icp_max_correspondence_distance").as_double();
        
        laser_yaw_offset_ = this->get_parameter("laser_yaw_offset").as_double();
        yaw_search_range_deg_ = this->get_parameter("yaw_search_range_deg").as_double();
        yaw_search_step_deg_ = this->get_parameter("yaw_search_step_deg").as_double();
        coarse_icp_distance_ = this->get_parameter("coarse_icp_distance").as_double();
        
        has_map_ = false;
        has_charger_pose_ = false;
        
        // Subscribers
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            map_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
            std::bind(&ScanMatcherService::map_callback, this, std::placeholders::_1));
            
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic_, 10,
            std::bind(&ScanMatcherService::scan_callback, this, std::placeholders::_1));
            
        charger_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            charger_pose_topic_, 10,
            std::bind(&ScanMatcherService::charger_pose_callback, this, std::placeholders::_1));
            
        // Publishers
        initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            output_pose_topic_, rclcpp::QoS(1).transient_local());
            
        // Service
        init_pose_srv_ = this->create_service<asump_localization::srv::InitializePose>(
            "initialize_pose",
            std::bind(&ScanMatcherService::initialize_pose_callback, this, 
                      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
                      
        RCLCPP_INFO(get_logger(), "=== SERVICE READY. Waiting for calls on /initialize_pose ===");
    }

private:
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        
        int width = msg->info.width;
        int height = msg->info.height;
        float res = msg->info.resolution;
        float ox = msg->info.origin.position.x;
        float oy = msg->info.origin.position.y;
        
        for (int r = 0; r < height; ++r) {
            for (int c = 0; c < width; ++c) {
                int idx = r * width + c;
                if (msg->data[idx] > map_occupied_threshold_) { 
                    pcl::PointXYZ p;
                    p.x = ox + (c + 0.5) * res;
                    p.y = oy + (r + 0.5) * res;
                    p.z = 0.0;
                    map_cloud->push_back(p);
                }
            }
        }
        
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(map_cloud);
        vg.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
        map_filtered_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        vg.filter(*map_filtered_);
        
        has_map_ = true;
        RCLCPP_INFO(get_logger(), "Map processed: %zu filtered points", map_filtered_->size());
    }
    
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_scan_ = msg;
    }
    
    void charger_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        charger_pose_ = *msg;
        has_charger_pose_ = true;
    }
    
    void initialize_pose_callback(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<asump_localization::srv::InitializePose::Request> request,
        std::shared_ptr<asump_localization::srv::InitializePose::Response> response)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        
        if (!has_map_ || !last_scan_ || !map_filtered_ || map_filtered_->empty()) {
            response->success = false;
            response->message = "Map or Scan not available yet.";
            return;
        }
        
        if (request->use_3d_refinement) {
            RCLCPP_WARN(get_logger(), "3D refinement is ignored in this 2D scan matcher.");
        }
        
        // 1. Determine seed pose (initial guess)
        Eigen::Affine3d seed_pose = Eigen::Affine3d::Identity();
        bool has_seed = false;
        
        if (request->initial_guess.header.frame_id != "") {
            seed_pose.translation() = Eigen::Vector3d(
                request->initial_guess.pose.pose.position.x,
                request->initial_guess.pose.pose.position.y,
                0.0);
            Eigen::Quaterniond q(
                request->initial_guess.pose.pose.orientation.w,
                request->initial_guess.pose.pose.orientation.x,
                request->initial_guess.pose.pose.orientation.y,
                request->initial_guess.pose.pose.orientation.z);
            seed_pose.linear() = q.toRotationMatrix();
            has_seed = true;
            RCLCPP_INFO(get_logger(), "Using provided initial_guess as seed.");
        } else if (has_charger_pose_) {
            seed_pose.translation() = Eigen::Vector3d(
                charger_pose_.pose.position.x,
                charger_pose_.pose.position.y,
                0.0);
            Eigen::Quaterniond q(
                charger_pose_.pose.orientation.w,
                charger_pose_.pose.orientation.x,
                charger_pose_.pose.orientation.y,
                charger_pose_.pose.orientation.z);
            seed_pose.linear() = q.toRotationMatrix();
            has_seed = true;
            RCLCPP_INFO(get_logger(), "Using charger_pose as seed.");
        }
        
        if (!has_seed) {
            response->success = false;
            response->message = "No initial_guess provided and no charger_pose available.";
            return;
        }
        
        enforce2DPose(seed_pose);
        
        // 2. Prepare Scan Cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        for (size_t i = 0; i < last_scan_->ranges.size(); ++i) {
            float range = last_scan_->ranges[i];
            if (range > last_scan_->range_min && range < last_scan_->range_max && std::isfinite(range)) {
                // Учитываем физический поворот лидара
                float angle = last_scan_->angle_min + i * last_scan_->angle_increment + laser_yaw_offset_;
                pcl::PointXYZ p;
                p.x = range * cos(angle);
                p.y = range * sin(angle);
                p.z = 0.0;
                scan_cloud->push_back(p);
            }
        }
        
        if (scan_cloud->empty()) {
            response->success = false;
            response->message = "Scan cloud is empty.";
            return;
        }
        
        // Downsample scan
        pcl::VoxelGrid<pcl::PointXYZ> vg_scan;
        vg_scan.setInputCloud(scan_cloud);
        vg_scan.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
        pcl::PointCloud<pcl::PointXYZ>::Ptr scan_filtered(new pcl::PointCloud<pcl::PointXYZ>());
        vg_scan.filter(*scan_filtered);
        
        // 3. Run ICP with Yaw Search and Coarse-to-Fine strategy
        double yaw_search_range = yaw_search_range_deg_ * M_PI / 180.0;
        double yaw_search_step = yaw_search_step_deg_ * M_PI / 180.0;
        
        Eigen::Quaterniond q_seed(seed_pose.linear());
        double seed_yaw = std::atan2(2.0 * (q_seed.w() * q_seed.z() + q_seed.x() * q_seed.y()),
                                     1.0 - 2.0 * (q_seed.y() * q_seed.y() + q_seed.z() * q_seed.z()));
        
        double best_fitness = std::numeric_limits<double>::max();
        Eigen::Affine3d best_final_pose = seed_pose;
        bool any_converged = false;
        
        // Setup Coarse ICP (большой радиус, чтобы "захватить" правильные стены)
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_coarse;
        icp_coarse.setInputSource(scan_filtered);
        icp_coarse.setInputTarget(map_filtered_);
        icp_coarse.setMaximumIterations(15);
        icp_coarse.setMaxCorrespondenceDistance(coarse_icp_distance_);
        icp_coarse.setTransformationEpsilon(1e-5);
        
        // Setup Fine ICP (малый радиус для точной подгонки)
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_fine;
        icp_fine.setInputSource(scan_filtered);
        icp_fine.setInputTarget(map_filtered_);
        icp_fine.setMaximumIterations(max_icp_iterations_);
        icp_fine.setMaxCorrespondenceDistance(icp_max_correspondence_distance_);
        icp_fine.setTransformationEpsilon(1e-8);
        icp_fine.setEuclideanFitnessEpsilon(1e-6);
        
        RCLCPP_INFO(get_logger(), "Starting Yaw Search from %.1f to %.1f deg...", 
                    -yaw_search_range_deg_, yaw_search_range_deg_);
                    
        for (double dyaw = -yaw_search_range; dyaw <= yaw_search_range + 1e-4; dyaw += yaw_search_step) {
            Eigen::Affine3d hypothesis = seed_pose;
            double test_yaw = seed_yaw + dyaw;
            Eigen::Quaterniond q_test(Eigen::AngleAxisd(test_yaw, Eigen::Vector3d::UnitZ()));
            hypothesis.linear() = q_test.toRotationMatrix();
            
            pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_coarse(new pcl::PointCloud<pcl::PointXYZ>());
            icp_coarse.align(*aligned_coarse, hypothesis.matrix().cast<float>());
            
            if (icp_coarse.hasConverged()) {
                Eigen::Matrix4f coarse_transform = icp_coarse.getFinalTransformation();
                
                pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_fine(new pcl::PointCloud<pcl::PointXYZ>());
                icp_fine.align(*aligned_fine, coarse_transform);
                
                if (icp_fine.hasConverged()) {
                    double fitness = icp_fine.getFitnessScore();
                    if (fitness < best_fitness) {
                        best_fitness = fitness;
                        best_final_pose = Eigen::Affine3d(icp_fine.getFinalTransformation().cast<double>());
                        any_converged = true;
                    }
                }
            }
        }
        
        if (any_converged) {
            enforce2DPose(best_final_pose);
            
            RCLCPP_INFO(get_logger(), "ICP converged! Best Fitness: %f", best_fitness);
            
            // 4. Publish to topic
            geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
            pose_msg.header.stamp = this->now();
            pose_msg.header.frame_id = "map";
            
            pose_msg.pose.pose.position.x = best_final_pose.translation().x();
            pose_msg.pose.pose.position.y = best_final_pose.translation().y();
            pose_msg.pose.pose.position.z = 0.0;
            
            Eigen::Quaterniond q(best_final_pose.linear());
            pose_msg.pose.pose.orientation.x = q.x();
            pose_msg.pose.pose.orientation.y = q.y();
            pose_msg.pose.pose.orientation.z = q.z();
            pose_msg.pose.pose.orientation.w = q.w();
            
            // Set covariance based on fitness score
            double cov = std::min(0.5, best_fitness * 0.1); // simple heuristic
            pose_msg.pose.covariance[0] = cov;  // x
            pose_msg.pose.covariance[7] = cov;  // y
            pose_msg.pose.covariance[35] = cov * 0.5; // yaw
            
            initial_pose_pub_->publish(pose_msg);
            
            // 5. Fill response
            response->pose = pose_msg;
            response->success = true;
            response->message = "ICP converged and pose published to " + output_pose_topic_;
            
        } else {
            RCLCPP_WARN(get_logger(), "ICP did not converge for any yaw hypothesis.");
            response->success = false;
            response->message = "ICP did not converge.";
        }
    }
    
    void enforce2DPose(Eigen::Affine3d& pose) {
        Eigen::Vector3d trans = pose.translation();
        trans.z() = 0.0;
        
        Eigen::Matrix3d rot = pose.linear();
        Eigen::Vector3d x_axis = rot.col(0);
        x_axis.z() = 0.0;
        if (x_axis.norm() > 1e-6) x_axis.normalize();
        else x_axis = Eigen::Vector3d::UnitX();
        
        Eigen::Vector3d y_axis = Eigen::Vector3d::UnitZ().cross(x_axis);
        
        rot.col(0) = x_axis;
        rot.col(1) = y_axis;
        rot.col(2) = Eigen::Vector3d::UnitZ();
        
        pose.linear() = rot;
        pose.translation() = trans;
    }

    // Subscribers & Publishers
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr charger_pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
    rclcpp::Service<asump_localization::srv::InitializePose>::SharedPtr init_pose_srv_;
    
    // State
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
    geometry_msgs::msg::PoseStamped charger_pose_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_filtered_;
    bool has_map_;
    bool has_charger_pose_;
    std::mutex mutex_;
    
    // Parameters
    std::string map_topic_;
    std::string scan_topic_;
    std::string charger_pose_topic_;
    std::string output_pose_topic_;
    double voxel_size_;
    int map_occupied_threshold_;
    int max_icp_iterations_;
    double icp_max_correspondence_distance_;
    
    double laser_yaw_offset_;
    double yaw_search_range_deg_;
    double yaw_search_step_deg_;
    double coarse_icp_distance_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ScanMatcherService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}