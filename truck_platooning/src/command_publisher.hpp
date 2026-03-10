#ifndef COMMAND_PUBLISHER_HPP
#define COMMAND_PUBLISHER_HPP

#include <vector>
#include <memory>
#include <cmath>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

void publish_commands(
    const std::vector<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr>& throttle_publishers,
    const std::vector<float>& current_velocities,
    float target_velocity,
    const std::vector<float>& last_steering,
    bool emergency_stop = false
) {
    for (size_t truck_id = 0; truck_id < throttle_publishers.size(); ++truck_id) {
        std_msgs::msg::Float32 msg;

        if (truck_id == 0 && emergency_stop) {
            msg.data = -1.0f;  // throttle = 0
            throttle_publishers[truck_id]->publish(msg);
            continue;
        }

        const float velocity_error = current_velocities[truck_id] - target_velocity;
        float throttle_value = 0.8f;
        if (target_velocity <= 0.1f && current_velocities[truck_id] > 1.0f) {
            throttle_value = -1.0f;
        } else if (velocity_error > 0.5f) {
            // Overspeed 구간은 coast(0.0)가 아니라 능동 제동으로 감속 응답성을 높인다.
            throttle_value = std::clamp(-0.22f * velocity_error, -1.0f, 0.0f);
        }
        if (std::abs(last_steering[truck_id]) > 3.0f) {
            throttle_value *= 0.8f;
        }

        msg.data = throttle_value;
        throttle_publishers[truck_id]->publish(msg);
    }
}

#endif // COMMAND_PUBLISHER_HPP
