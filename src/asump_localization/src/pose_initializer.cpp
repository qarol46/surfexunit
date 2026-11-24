#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <Eigen/Dense>
#include <mutex>
#include <memory>
#include <string>
#include <cmath>
#include <random>
#include <vector>
#include <chrono>
#include <thread>

#include "asump_localization/srv/initialize_pose.hpp"

struct Particle {
    double x, y, theta;
    double weight;
};

class ParticleFilterInitializer : public rclcpp::Node
{
public:
    ParticleFilterInitializer() : Node("particle_filter_initializer"), gen_(std::random_device{}())
    {
        RCLCPP_INFO(get_logger(), "=== PARTICLE FILTER INITIALIZER STARTING ===");

        this->declare_parameter("map_topic", "/map");
        this->declare_parameter("scan_topic", "/scan");
        this->declare_parameter("charger_pose_topic", "/zones/charger_pose");
        this->declare_parameter("stairs_pose_topic", "/zones/stairs_pose");
        this->declare_parameter("output_pose_topic", "/initialpose");
        this->declare_parameter("particles_topic", "/particles");

        this->declare_parameter("num_particles", 3000);
        this->declare_parameter("convergence_threshold", 0.3);
        this->declare_parameter("max_iterations", 200);
        this->declare_parameter("publish_particles", true);
        this->declare_parameter("random_particle_ratio", 0.1);
        this->declare_parameter("sigma_hit", 0.15);
        this->declare_parameter("z_hit", 0.95);
        this->declare_parameter("z_rand", 0.05);
        this->declare_parameter("beam_step", 3);
        this->declare_parameter("initialization_timeout_sec", 10.0);

        // Радиус локальной области инициализации (вокруг ориентира:
        // зарядной станции, лестницы или явно заданной позы), метры
        this->declare_parameter("search_radius", 3.0);

        map_topic_ = this->get_parameter("map_topic").as_string();
        scan_topic_ = this->get_parameter("scan_topic").as_string();
        charger_pose_topic_ = this->get_parameter("charger_pose_topic").as_string();
        stairs_pose_topic_ = this->get_parameter("stairs_pose_topic").as_string();
        search_radius_ = this->get_parameter("search_radius").as_double();
        output_pose_topic_ = this->get_parameter("output_pose_topic").as_string();
        particles_topic_ = this->get_parameter("particles_topic").as_string();

        num_particles_ = this->get_parameter("num_particles").as_int();
        convergence_threshold_ = this->get_parameter("convergence_threshold").as_double();
        max_iterations_ = this->get_parameter("max_iterations").as_int();
        publish_particles_ = this->get_parameter("publish_particles").as_bool();
        random_particle_ratio_ = this->get_parameter("random_particle_ratio").as_double();
        sigma_hit_ = this->get_parameter("sigma_hit").as_double();
        z_hit_ = this->get_parameter("z_hit").as_double();
        z_rand_ = this->get_parameter("z_rand").as_double();
        beam_step_ = this->get_parameter("beam_step").as_int();
        initialization_timeout_sec_ = this->get_parameter("initialization_timeout_sec").as_double();

        has_map_ = false;
        has_charger_pose_ = false;
        is_initializing_ = false;

        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            map_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
            std::bind(&ParticleFilterInitializer::map_callback, this, std::placeholders::_1));

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic_, 10,
            std::bind(&ParticleFilterInitializer::scan_callback, this, std::placeholders::_1));

        charger_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            charger_pose_topic_, 10,
            std::bind(&ParticleFilterInitializer::charger_pose_callback, this, std::placeholders::_1));

        stairs_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            stairs_pose_topic_, 10,
            std::bind(&ParticleFilterInitializer::stairs_pose_callback, this, std::placeholders::_1));

        initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            output_pose_topic_, rclcpp::QoS(1).transient_local());

        if (publish_particles_) {
            particles_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
                particles_topic_, 10);
        }

        init_pose_srv_ = this->create_service<asump_localization::srv::InitializePose>(
            "initialize_pose",
            std::bind(&ParticleFilterInitializer::initialize_pose_callback, this,
                      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        RCLCPP_INFO(get_logger(), "=== SERVICE READY. Waiting for calls on /initialize_pose ===");
    }

private:
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        
        RCLCPP_INFO(get_logger(), "Received map: %dx%d, res=%.3f",
                    msg->info.width, msg->info.height, msg->info.resolution);

        map_info_ = msg->info;

        map_occupancy_ = cv::Mat(msg->info.height, msg->info.width, CV_8UC1, cv::Scalar(0));
        for (size_t i = 0; i < msg->data.size(); ++i) {
            if (msg->data[i] > 50) {
                map_occupancy_.data[i] = 255;
            }
        }

        cv::Mat obstacles = 255 - map_occupancy_;
        cv::distanceTransform(obstacles, dist_field_, cv::DIST_L2, cv::DIST_MASK_5);

        has_map_ = true;
        RCLCPP_INFO(get_logger(), "Distance field computed.");
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        last_scan_ = msg;
    }

    void charger_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        charger_pose_ = *msg;
        has_charger_pose_ = true;
    }

    void stairs_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stairs_pose_ = *msg;
        has_stairs_pose_ = true;
    }

    void initialize_pose_callback(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<asump_localization::srv::InitializePose::Request> request,
        std::shared_ptr<asump_localization::srv::InitializePose::Response> response)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (is_initializing_) {
            response->success = false;
            response->message = "Initialization already in progress.";
            return;
        }

        if (!has_map_) {
            response->success = false;
            response->message = "Map not available yet.";
            return;
        }

        if (!last_scan_) {
            response->success = false;
            response->message = "No scan available yet.";
            return;
        }

        if (request->use_3d_refinement) {
            RCLCPP_WARN(get_logger(), "3D refinement is ignored in this 2D particle filter.");
        }

        is_initializing_ = true;
        RCLCPP_INFO(get_logger(), "Starting particle filter initialization...");

        double seed_x = 0, seed_y = 0, seed_theta = 0;
        bool has_seed = false;

        // frame_id в initial_guess выбирает область инициализации:
        //   "charger" -> круг search_radius_ вокруг зарядной станции
        //   "stairs"  -> круг search_radius_ вокруг лестницы
        //   прочее непустое -> явная поза в системе map (круг вокруг неё)
        //   пусто           -> глобальный поиск по всей карте
        const std::string& area = request->initial_guess.header.frame_id;

        auto yaw_of = [](const geometry_msgs::msg::Pose& pose) {
            Eigen::Quaterniond q(
                pose.orientation.w, pose.orientation.x,
                pose.orientation.y, pose.orientation.z);
            Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
            return euler[0];
        };

        if (area == "charger" || area == "stairs") {
            const bool want_charger = (area == "charger");
            if (want_charger && !has_charger_pose_) {
                response->success = false;
                response->message = "Charger pose not available yet.";
                return;
            }
            if (!want_charger && !has_stairs_pose_) {
                response->success = false;
                response->message = "Stairs pose not available yet.";
                return;
            }
            const auto& landmark = want_charger ? charger_pose_ : stairs_pose_;
            seed_x = landmark.pose.position.x;
            seed_y = landmark.pose.position.y;
            seed_theta = yaw_of(landmark.pose);
            has_seed = true;
            RCLCPP_INFO(get_logger(), "Using %s pose as seed: x=%.2f, y=%.2f, theta=%.2f",
                        area.c_str(), seed_x, seed_y, seed_theta);
        } else if (!area.empty()) {
            seed_x = request->initial_guess.pose.pose.position.x;
            seed_y = request->initial_guess.pose.pose.position.y;
            seed_theta = yaw_of(request->initial_guess.pose.pose);
            has_seed = true;
            RCLCPP_INFO(get_logger(), "Using provided initial_guess as seed: x=%.2f, y=%.2f, theta=%.2f",
                        seed_x, seed_y, seed_theta);
        } else {
            RCLCPP_INFO(get_logger(), "No area specified, global search across whole map.");
        }

        initializeParticles(has_seed, seed_x, seed_y, seed_theta);

        auto start_time = std::chrono::steady_clock::now();
        auto timeout = std::chrono::duration<double>(initialization_timeout_sec_);

        bool converged = false;
        int iteration = 0;

        while (iteration < max_iterations_) {
            auto current_time = std::chrono::steady_clock::now();
            if (current_time - start_time > timeout) {
                RCLCPP_WARN(get_logger(), "Initialization timeout reached.");
                break;
            }

            if (!last_scan_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            updateWeights(last_scan_);

            if (publish_particles_) {
                publishParticles();
            }

            if (checkConvergence()) {
                converged = true;
                RCLCPP_INFO(get_logger(), "Particles converged after %d iterations.", iteration + 1);
                break;
            }

            resample();
            iteration++;

            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        double final_x, final_y, final_theta;
        double var_x, var_y, var_theta;
        computeFinalPose(final_x, final_y, final_theta, var_x, var_y, var_theta);

        if (converged || iteration > 0) {
            // Publish pose
            geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
            pose_msg.header.stamp = this->now();
            pose_msg.header.frame_id = "map";
            pose_msg.pose.pose.position.x = final_x;
            pose_msg.pose.pose.position.y = final_y;
            pose_msg.pose.pose.position.z = 0.0;
            
            pose_msg.pose.pose.orientation.z = std::sin(final_theta / 2.0);
            pose_msg.pose.pose.orientation.w = std::cos(final_theta / 2.0);

            pose_msg.pose.covariance[0] = std::max(var_x, 0.01);
            pose_msg.pose.covariance[7] = std::max(var_y, 0.01);
            pose_msg.pose.covariance[14] = 0.01;
            pose_msg.pose.covariance[21] = 0.01;
            pose_msg.pose.covariance[28] = 0.01;
            pose_msg.pose.covariance[35] = std::max(var_theta, 0.01);

            initial_pose_pub_->publish(pose_msg);

            response->pose = pose_msg;
            response->success = true;
            response->message = converged ? 
                "Particle filter converged and pose published." :
                "Particle filter completed (max iterations) and pose published.";

            RCLCPP_INFO(get_logger(), "Published: x=%.2f y=%.2f theta=%.2f (std: %.3f %.3f %.3f)",
                        final_x, final_y, final_theta, 
                        std::sqrt(var_x), std::sqrt(var_y), std::sqrt(var_theta));
        } else {
            response->success = false;
            response->message = "Particle filter failed to initialize.";
            RCLCPP_ERROR(get_logger(), "Particle filter failed to initialize.");
        }

        is_initializing_ = false;
    }

    void initializeParticles(bool has_seed, double seed_x, double seed_y, double seed_theta)
    {
        particles_.clear();
        particles_.reserve(num_particles_);

        // Локальная область поиска: круг search_radius_ вокруг seed-позы.
        // Запоминаем её — случайные частицы при ресемплинге берутся только
        // из неё, иначе фильтр может сойтись в похожем месте карты.
        area_active_ = has_seed;
        area_x_ = seed_x;
        area_y_ = seed_y;

        double map_min_x = map_info_.origin.position.x;
        double map_max_x = map_info_.origin.position.x + map_info_.width * map_info_.resolution;
        double map_min_y = map_info_.origin.position.y;
        double map_max_y = map_info_.origin.position.y + map_info_.height * map_info_.resolution;

        std::uniform_real_distribution<> dmx(map_min_x, map_max_x);
        std::uniform_real_distribution<> dmy(map_min_y, map_max_y);
        std::uniform_real_distribution<> d01(0.0, 1.0);
        std::uniform_real_distribution<> dt(-M_PI, M_PI);
        std::normal_distribution<> dtheta(0.0, 0.3);

        int placed = 0;
        int attempts = 0;
        const int max_attempts = num_particles_ * 100;

        while (placed < num_particles_ && attempts < max_attempts) {
            double x, y;
            if (area_active_) {
                // Равномерно по площади круга (sqrt для радиуса)
                double r = search_radius_ * std::sqrt(d01(gen_));
                double a = 2.0 * M_PI * d01(gen_);
                x = area_x_ + r * std::cos(a);
                y = area_y_ + r * std::sin(a);
            } else {
                x = dmx(gen_);
                y = dmy(gen_);
            }

            if (isFree(x, y)) {
                double theta = area_active_ ? seed_theta + dtheta(gen_) : dt(gen_);
                particles_.push_back({x, y, theta, 1.0 / num_particles_});
                placed++;
            }
            attempts++;
        }

        iteration_count_ = 0;
        if (area_active_) {
            RCLCPP_INFO(get_logger(),
                "Placed %d particles in radius %.1f m around (%.2f, %.2f) (%d attempts)",
                placed, search_radius_, area_x_, area_y_, attempts);
        } else {
            RCLCPP_INFO(get_logger(), "Placed %d particles across whole map (%d attempts)",
                placed, attempts);
        }

        if (placed == 0) {
            RCLCPP_ERROR(get_logger(), "Failed to place any particles!");
        }
    }

    bool isFree(double x, double y) const
    {
        int px = static_cast<int>((x - map_info_.origin.position.x) / map_info_.resolution);
        int py = static_cast<int>((y - map_info_.origin.position.y) / map_info_.resolution);
        if (px < 0 || px >= map_occupancy_.cols || py < 0 || py >= map_occupancy_.rows)
            return false;
        return map_occupancy_.at<uchar>(py, px) == 0;
    }

    double wallDistanceMeters(int px, int py) const
    {
        if (px < 0 || px >= dist_field_.cols || py < 0 || py >= dist_field_.rows)
            return -1.0;
        return static_cast<double>(dist_field_.at<float>(py, px)) * map_info_.resolution;
    }

    bool isWall(int px, int py) const
    {
        if (px < 0 || px >= map_occupancy_.cols || py < 0 || py >= map_occupancy_.rows)
            return true;
        return map_occupancy_.at<uchar>(py, px) == 255;
    }

    void updateWeights(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        double best_log_w = -1e30;

        for (auto& p : particles_) {
            double log_w = 0.0;
            int hits = 0;

            for (size_t i = 0; i < msg->ranges.size(); i += beam_step_) {
                float range = msg->ranges[i];
                if (!std::isfinite(range) || range < msg->range_min) continue;
                if (range > msg->range_max) range = msg->range_max;

                double angle = msg->angle_min + i * msg->angle_increment + p.theta;
                double ex = p.x + range * std::cos(angle);
                double ey = p.y + range * std::sin(angle);

                int px = static_cast<int>((ex - map_info_.origin.position.x) / map_info_.resolution);
                int py = static_cast<int>((ey - map_info_.origin.position.y) / map_info_.resolution);

                double prob;
                if (isWall(px, py)) {
                    prob = 1e-6;
                } else {
                    double d = wallDistanceMeters(px, py);
                    if (d < 0) {
                        prob = z_rand_ / msg->range_max;
                    } else {
                        double gauss = std::exp(-(d * d) / (2.0 * sigma_hit_ * sigma_hit_));
                        prob = z_hit_ * gauss + z_rand_ / msg->range_max;
                    }
                }
                log_w += std::log(std::max(prob, 1e-30));
                hits++;
            }

            p.weight = (hits > 0) ? (log_w / hits) : -1e10;
            best_log_w = std::max(best_log_w, p.weight);
        }

        double sum_w = 0.0;
        for (auto& p : particles_) {
            p.weight = std::exp(p.weight - best_log_w);
            sum_w += p.weight;
        }
        if (sum_w > 0) {
            for (auto& p : particles_) p.weight /= sum_w;
        }
    }

    bool checkConvergence()
    {
        double mx = 0, my = 0, ms = 0, mc = 0;
        for (const auto& p : particles_) {
            mx += p.x * p.weight;
            my += p.y * p.weight;
            ms += std::sin(p.theta) * p.weight;
            mc += std::cos(p.theta) * p.weight;
        }
        double vx = 0, vy = 0;
        for (const auto& p : particles_) {
            vx += p.weight * (p.x - mx) * (p.x - mx);
            vy += p.weight * (p.y - my) * (p.y - my);
        }
        double sx = std::sqrt(vx);
        double sy = std::sqrt(vy);
        double circ_var = 1.0 - std::sqrt(ms * ms + mc * mc);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Iter %d: std_x=%.3f, std_y=%.3f, circ_var=%.3f",
                             iteration_count_, sx, sy, circ_var);

        return (sx < convergence_threshold_ && sy < convergence_threshold_ && circ_var < 0.15);
    }

    void resample()
    {
        std::vector<Particle> next;
        next.reserve(num_particles_);

        double ratio = random_particle_ratio_ * std::max(0.2, 1.0 - (double)iteration_count_ / max_iterations_);
        int n_rand = static_cast<int>(num_particles_ * ratio);
        int n_resample = num_particles_ - n_rand;

        std::uniform_real_distribution<> u01(0.0, 1.0);
        double step = 1.0 / n_resample;
        double r = u01(gen_) * step;
        double cum = particles_[0].weight;
        int idx = 0;

        std::normal_distribution<> noise_xy(0.0, 0.03);
        std::normal_distribution<> noise_th(0.0, 0.02);

        for (int m = 0; m < n_resample; ++m) {
            double u = r + m * step;
            while (u > cum && idx < (int)particles_.size() - 1) {
                idx++;
                cum += particles_[idx].weight;
            }
            Particle np = particles_[idx];
            np.x += noise_xy(gen_);
            np.y += noise_xy(gen_);
            np.theta += noise_th(gen_);
            np.weight = 1.0 / num_particles_;

            if (isFree(np.x, np.y))
                next.push_back(np);
            else
                addRandom(next);
        }

        for (int m = 0; m < n_rand; ++m) addRandom(next);

        particles_ = std::move(next);
        iteration_count_++;
    }

    void addRandom(std::vector<Particle>& out)
    {
        // Случайные частицы добавляются только внутри активной области
        // поиска (круг search_radius_ вокруг seed-позы). Раньше они брались
        // со всей карты и на симметричном складе выигрывали в ложном месте.
        double map_min_x = map_info_.origin.position.x;
        double map_max_x = map_info_.origin.position.x + map_info_.width * map_info_.resolution;
        double map_min_y = map_info_.origin.position.y;
        double map_max_y = map_info_.origin.position.y + map_info_.height * map_info_.resolution;

        std::uniform_real_distribution<> dmx(map_min_x, map_max_x);
        std::uniform_real_distribution<> dmy(map_min_y, map_max_y);
        std::uniform_real_distribution<> d01(0.0, 1.0);
        std::uniform_real_distribution<> dt(-M_PI, M_PI);

        double x = 0.0, y = 0.0;
        for (int t = 0; t < 50; ++t) {
            if (area_active_) {
                double r = search_radius_ * std::sqrt(d01(gen_));
                double a = 2.0 * M_PI * d01(gen_);
                x = area_x_ + r * std::cos(a);
                y = area_y_ + r * std::sin(a);
            } else {
                x = dmx(gen_);
                y = dmy(gen_);
            }

            if (isFree(x, y)) {
                out.push_back({x, y, dt(gen_), 1.0 / num_particles_});
                return;
            }
        }
        out.push_back({x, y, dt(gen_), 1.0 / num_particles_});
    }

    void computeFinalPose(double& x, double& y, double& theta, double& var_x, double& var_y, double& var_theta)
    {
        double mx = 0, my = 0, ms = 0, mc = 0;
        for (const auto& p : particles_) {
            mx += p.x * p.weight;
            my += p.y * p.weight;
            ms += std::sin(p.theta) * p.weight;
            mc += std::cos(p.theta) * p.weight;
        }
        theta = std::atan2(ms, mc);

        double vx = 0, vy = 0, vt = 0;
        for (const auto& p : particles_) {
            vx += p.weight * (p.x - mx) * (p.x - mx);
            vy += p.weight * (p.y - my) * (p.y - my);
            double dt = p.theta - theta;
            while (dt > M_PI) dt -= 2 * M_PI;
            while (dt < -M_PI) dt += 2 * M_PI;
            vt += p.weight * dt * dt;
        }

        x = mx;
        y = my;
        var_x = vx;
        var_y = vy;
        var_theta = vt;
    }

    void publishParticles()
    {
        geometry_msgs::msg::PoseArray arr;
        arr.header.stamp = this->now();
        arr.header.frame_id = "map";
        arr.poses.reserve(particles_.size());
        for (const auto& p : particles_) {
            geometry_msgs::msg::Pose pose;
            pose.position.x = p.x;
            pose.position.y = p.y;
            pose.orientation.z = std::sin(p.theta / 2.0);
            pose.orientation.w = std::cos(p.theta / 2.0);
            arr.poses.push_back(pose);
        }
        particles_pub_->publish(arr);
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr charger_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr stairs_pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particles_pub_;
    rclcpp::Service<asump_localization::srv::InitializePose>::SharedPtr init_pose_srv_;

    nav_msgs::msg::MapMetaData map_info_;
    cv::Mat map_occupancy_;
    cv::Mat dist_field_;
    bool has_map_;

    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
    geometry_msgs::msg::PoseStamped charger_pose_;
    bool has_charger_pose_;
    geometry_msgs::msg::PoseStamped stairs_pose_;
    bool has_stairs_pose_ = false;

    // Активная локальная область поиска (круг вокруг seed-позы)
    bool area_active_ = false;
    double area_x_ = 0.0;
    double area_y_ = 0.0;

    std::vector<Particle> particles_;
    int iteration_count_ = 0;
    bool is_initializing_;
    std::mt19937 gen_;

    std::mutex mutex_;

    // Parameters
    std::string map_topic_;
    std::string scan_topic_;
    std::string charger_pose_topic_;
    std::string stairs_pose_topic_;
    double search_radius_;
    std::string output_pose_topic_;
    std::string particles_topic_;

    int num_particles_;
    double convergence_threshold_;
    int max_iterations_;
    bool publish_particles_;
    double random_particle_ratio_;
    double sigma_hit_;
    double z_hit_;
    double z_rand_;
    int beam_step_;
    double initialization_timeout_sec_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ParticleFilterInitializer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}