#ifndef PLATOONING_MANAGER_HPP
#define PLATOONING_MANAGER_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include <algorithm>
#include <memory>
#include <optional>
#include <string>

class PlatooningManager {
public:
    PlatooningManager(const std::shared_ptr<rclcpp::Node> &node, const std::string &namespace_)
        : integral_(0.0f),
          prev_error_(0.0f),
          prev_time_(node->get_clock()->now()),
          node_(node),
          namespace_(namespace_),
          target_distance_(12.0f),
          safe_distance_(4.5f),
          kp_(0.3f),
          ki_(0.01f),
          kd_(1.8f) {
        throttle_pub_ = node_->create_publisher<std_msgs::msg::Float32>(
            "/" + namespace_ + "/throttle_control", 10);
    }

    void update_distance(float lidar_distance, bool emergency_stop = false) {
        lidar_distance_ = lidar_distance;
        control_speed(emergency_stop);
    }

    float integral_;
    float prev_error_;
    rclcpp::Time prev_time_;

private:
    void control_speed(bool emergency_stop = false) {
        if (emergency_stop) {
            std_msgs::msg::Float32 msg;
            msg.data = -1.0f;
            throttle_pub_->publish(msg);
            return;
        }

        if (!lidar_distance_.has_value()) {
            RCLCPP_WARN(node_->get_logger(), "[%s] LiDAR 거리 정보 없음, 제어 수행 불가", namespace_.c_str());
            return;
        }

        const rclcpp::Time current_time = node_->get_clock()->now();
        double dt = (current_time - prev_time_).seconds();
        if (dt <= 0.0) dt = 1.0;

        const float distance_error = *lidar_distance_ - target_distance_;
        integral_ += distance_error * static_cast<float>(dt);
        const float derivative = (distance_error - prev_error_) / static_cast<float>(dt);
        float speed_command = kp_ * distance_error + ki_ * integral_ + kd_ * derivative;

        if (*lidar_distance_ < safe_distance_) {
            speed_command = -1.0f;
        }
        speed_command = std::clamp(speed_command, -1.0f, 1.0f);

        std_msgs::msg::Float32 msg;
        msg.data = speed_command;
        throttle_pub_->publish(msg);

        prev_error_ = distance_error;
        prev_time_ = current_time;
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::string namespace_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_pub_;
    std::optional<float> lidar_distance_;
    float target_distance_;
    float safe_distance_;
    float kp_;
    float ki_;
    float kd_;
};

#endif  // PLATOONING_MANAGER_HPP
