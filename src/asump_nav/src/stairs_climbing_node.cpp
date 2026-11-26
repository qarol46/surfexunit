#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;

class StairClimberNode : public rclcpp::Node
{
public:
    StairClimberNode() : Node("stair_climber_node")
    {
        // --- Параметры ---
        this->declare_parameter("max_depth_m", 3.0);       
        this->declare_parameter("align_kp", 1.5);          
        this->declare_parameter("align_tolerance_deg", 3.0);
        this->declare_parameter("climb_speed", -1.75);      
        this->declare_parameter("canny_low", 10);          // Нижний порог Canny
        this->declare_parameter("canny_high", 70);         // Верхний порог Canny
        this->declare_parameter("angle_tolerance_deg", 25.0); // Допуск по углу для горизонтальности

        // --- Подписчики ---
        image_sub_.subscribe(this, "/back_camera/image");
        depth_sub_.subscribe(this, "/back_camera/depth_image");
        info_sub_.subscribe(this, "/back_camera/camera_info");
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10, std::bind(&StairClimberNode::imuCallback, this, std::placeholders::_1));

        // --- Синхронизатор ---
        using SyncPolicy = message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>;
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), image_sub_, depth_sub_, info_sub_);
        sync_->registerCallback(&StairClimberNode::syncCallback, this);

        // --- Публикации ---
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        debug_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/stair_climber/debug_image", 10);

        // --- Таймер управления (20 Hz) ---
        control_timer_ = this->create_wall_timer(50ms, std::bind(&StairClimberNode::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "Stair Climber Node initialized. State: SEARCHING");
    }

    ~StairClimberNode() {
        stopRobot();
    }

private:
    enum class State { SEARCHING, ALIGNING, CLIMBING, DONE };
    State current_state_ = State::SEARCHING;

    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    std::shared_ptr<message_filters::Synchronizer<
        message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>>> sync_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_img_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // Данные IMU
    double current_pitch_ = 0.0;
    double start_pitch_ = 0.0;
    bool climb_started_ = false;
    rclcpp::Time climb_start_time_;
    rclcpp::Time last_flat_time_;

    // Визуальные данные
    cv::Mat latest_rgb_;
    double detected_stair_angle_deg_ = 0.0;
    bool stairs_detected_ = false;
    std::mutex data_mutex_;

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        std::lock_guard<std::mutex> lock(data_mutex_);
        current_pitch_ = pitch;
    }

    void syncCallback(const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,
                      const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
                      const sensor_msgs::msg::CameraInfo::ConstSharedPtr& /*info_msg*/)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (current_state_ == State::DONE) return;

        try {
            cv_bridge::CvImagePtr cv_rgb = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
            cv_bridge::CvImagePtr cv_depth = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
            
            latest_rgb_ = cv_rgb->image.clone();
            processDepthAndFindStairs(cv_depth->image, latest_rgb_);
            
            auto debug_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", latest_rgb_).toImageMsg();
            debug_msg->header = rgb_msg->header;
            debug_img_pub_->publish(*debug_msg);

        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void processDepthAndFindStairs(const cv::Mat& depth_32f, cv::Mat& rgb_out) {
        double max_depth = this->get_parameter("max_depth_m").as_double();
        int canny_low = this->get_parameter("canny_low").as_int();
        int canny_high = this->get_parameter("canny_high").as_int();
        double angle_tol = this->get_parameter("angle_tolerance_deg").as_double();

        cv::Mat valid_mask = (depth_32f > 0.1) & (depth_32f <= max_depth);

        cv::Mat depth_8u;
        depth_32f.convertTo(depth_8u, CV_8U, 255.0 / max_depth);
        depth_8u.setTo(0, ~valid_mask);

        cv::medianBlur(depth_8u, depth_8u, 5);

        cv::Mat edges;
        cv::Canny(depth_8u, edges, canny_low, canny_high);

        // Поиск линий с более мягкими параметрами
        std::vector<cv::Vec4i> lines;
        cv::HoughLinesP(edges, lines, 1, CV_PI / 180, 30, 30, 30);

        std::vector<double> horizontal_angles;
        stairs_detected_ = false;

        // Рисуем ВСЕ найденные линии КРАСНЫМ (для отладки)
        for (size_t i = 0; i < lines.size(); i++) {
            cv::Vec4i l = lines[i];
            cv::line(rgb_out, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 1);
        }

        // Теперь фильтруем и рисуем только горизонтальные ЗЕЛЕНЫМ
        for (size_t i = 0; i < lines.size(); i++) {
            cv::Vec4i l = lines[i];
            
            double angle_rad = atan2(l[3] - l[1], l[2] - l[0]);
            double angle_deg = angle_rad * 180.0 / CV_PI;
            
            if (angle_deg > 90) angle_deg -= 180;
            if (angle_deg < -90) angle_deg += 180;

            // Увеличили допуск по углу до 25 градусов
            if (abs(angle_deg) < angle_tol) { 
                horizontal_angles.push_back(angle_deg);
                cv::line(rgb_out, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 255, 0), 3);
            }
        }

        // Отладочный вывод
        RCLCPP_DEBUG(this->get_logger(), "Found %zu total lines, %zu horizontal lines", 
                    lines.size(), horizontal_angles.size());

        if (horizontal_angles.size() >= 2) { 
            stairs_detected_ = true;
            double sum = 0;
            for(double a : horizontal_angles) sum += a;
            detected_stair_angle_deg_ = sum / horizontal_angles.size();
            
            cv::putText(rgb_out, "Stairs: " + std::to_string(horizontal_angles.size()) + " lines. Angle: " + 
                        std::to_string(detected_stair_angle_deg_).substr(0, 4) + " deg", 
                        cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
        } else {
            stairs_detected_ = false;
            detected_stair_angle_deg_ = 0.0;
        }
    }

    void controlLoop() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        geometry_msgs::msg::Twist cmd;
        
        double kp = this->get_parameter("align_kp").as_double();
        double tol = this->get_parameter("align_tolerance_deg").as_double();
        double speed = this->get_parameter("climb_speed").as_double();

        switch (current_state_) {
            case State::SEARCHING:
                if (stairs_detected_) {
                    current_state_ = State::ALIGNING;
                    RCLCPP_INFO(this->get_logger(), "State -> ALIGNING");
                }
                break;

            case State::ALIGNING:
                if (!stairs_detected_) {
                    current_state_ = State::SEARCHING;
                    break;
                }
                
                cmd.angular.z = kp * (-detected_stair_angle_deg_ * M_PI / 180.0);
                
                if (abs(detected_stair_angle_deg_) < tol) {
                    cmd.angular.z = 0;
                    current_state_ = State::CLIMBING;
                    
                    start_pitch_ = current_pitch_;
                    climb_started_ = false;
                    last_flat_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
                    RCLCPP_INFO(this->get_logger(), "Aligned! State -> CLIMBING. Baseline pitch: %.3f rad", start_pitch_);
                }
                break;

            case State::CLIMBING:
                cmd.linear.x = speed;
                
                if (!climb_started_ && abs(current_pitch_ - start_pitch_) > 0.05) {
                    climb_started_ = true;
                    climb_start_time_ = this->now();
                    last_flat_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
                    RCLCPP_INFO(this->get_logger(), "IMU: Actual climbing started (Pitch: %.3f rad)", current_pitch_);
                }
                
                if (climb_started_) {
                    if (abs(current_pitch_ - start_pitch_) < 0.03) {
                        if (last_flat_time_.seconds() == 0) {
                            last_flat_time_ = this->now();
                            RCLCPP_INFO(this->get_logger(), "IMU: Pitch returned to baseline, starting flat timer...");
                        }
                        
                        double time_since_start = (this->now() - climb_start_time_).seconds();
                        double time_flat = (this->now() - last_flat_time_).seconds();
                        
                        if (time_since_start > 3.0 && time_flat > 1.0) {
                            current_state_ = State::DONE;
                            RCLCPP_INFO(this->get_logger(), "IMU: Reached the top! Pitch stable at %.3f rad for %.1f sec. State -> DONE", 
                                       current_pitch_, time_flat);
                        }
                    } else {
                        if (last_flat_time_.seconds() != 0) {
                            RCLCPP_INFO(this->get_logger(), "IMU: Pitch deviated from baseline (%.3f rad), resetting flat timer", current_pitch_);
                        }
                        last_flat_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
                    }
                }
                break;

            case State::DONE:
                break;
        }

        if (current_state_ == State::DONE) {
            stopRobot();
        } else {
            cmd_vel_pub_->publish(cmd);
        }
    }

    void stopRobot() {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StairClimberNode>());
    rclcpp::shutdown();
    return 0;
}