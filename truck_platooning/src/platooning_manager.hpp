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

    void update_distance(float lidar_distance, float leader_velocity, bool emergency_stop = false, float ego_velocity = 0.0f) {
        lidar_distance_ = lidar_distance;
        control_speed(leader_velocity, emergency_stop, ego_velocity);
    }

    float integral_;
    float prev_error_;
    rclcpp::Time prev_time_;

private:
    void control_speed(float leader_velocity, bool emergency_stop = false, float ego_velocity = 0.0f) {
        if (emergency_stop) {
            std_msgs::msg::Float32 msg;
            msg.data = -1.0f;
            throttle_pub_->publish(msg);
            return;
        }

        if (!lidar_distance_.has_value()) {
            return;
        }

        const rclcpp::Time current_time = node_->get_clock()->now();
        double dt = (current_time - prev_time_).seconds();
        if (dt <= 0.0) dt = 0.05;

        // 1. 거리 오차 기반 속도 보정치 계산 (PID)
        const float distance_error = *lidar_distance_ - target_distance_;
        integral_ += distance_error * static_cast<float>(dt);
        const float derivative = (distance_error - prev_error_) / static_cast<float>(dt);
        
        // 거리 유지를 위한 보정 속도 (m/s)
        float dist_correction = 0.5f * distance_error + 0.01f * integral_ + 0.1f * derivative;

        // 2. 최종 목표 속도 = 선행 차량 속도 + 거리 보정 속도
        float target_velocity = leader_velocity + dist_correction;

        // 3. 목표 속도 추종을 위한 스로틀 계산 (P 제어)
        float speed_command;
        if (ego_velocity > 0.1f || leader_velocity > 0.1f) {
            float vel_error = target_velocity - ego_velocity;
            speed_command = vel_error * 0.4f;
        } else {
            speed_command = dist_correction * 0.2f;
        }

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
