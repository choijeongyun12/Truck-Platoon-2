#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

#include "pid_controller.hpp"
#include "lane_detect.hpp"
#include "command_publisher.hpp"
#include "distance_sensor.hpp"
#include "platooning_manager.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

enum class ManeuverState {
    IDLE = 0,
    LEADER_EXITS_LANE = 1,
    LEADER_CREATES_GAP = 2,
    SUCCESSOR_ENTERS_GAP = 3,
    FOLLOWER_ENTERS_GAP = 4,
    REORDER_COMPLETE = 5,
    COOLDOWN = 6,
    LEADER_REENTERS_LANE = 7,
    PROMOTE_TARGET_EXITS = 21,
    PROMOTE_PLATOON_CREATES_GAP = 22,
    PROMOTE_TARGET_REENTERS = 23,
};

struct VehiclePose {
    bool valid = false;
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

struct PendingCommand {
    enum class Type { Reorder, Promote, Manual } type;
    int truck_id = -1;
    std::string direction;
};

class LaneFollowingNode : public rclcpp::Node {
public:
    LaneFollowingNode()
        : Node("lane_following_node"),
          truck_order_{0, 1, 2},
          current_target_lane_{"center", "center", "center"},
          transition_factor_{0.0f, 0.0f, 0.0f},
          current_velocities_{0.0f, 0.0f, 0.0f},
          last_steering_{0.0f, 0.0f, 0.0f},
          maneuver_state_(ManeuverState::IDLE),
          reorder_direction_("left") {
        const auto qos_camera =
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)).best_effort();
        const auto qos_odom =
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)).best_effort();
        const auto qos_pose =
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)).best_effort();

        for (int i = 0; i < 3; ++i) {
            const std::string ns = "truck" + std::to_string(i);
            steer_publishers_[i] = create_publisher<std_msgs::msg::Float32>("/" + ns + "/steer_control", 10);
            throttle_publishers_[i] = create_publisher<std_msgs::msg::Float32>("/" + ns + "/throttle_control", 10);

            velocity_subscribers_[i] = create_subscription<std_msgs::msg::Float32>(
                "/" + ns + "/velocity", 10,
                [this, i](const std_msgs::msg::Float32::SharedPtr msg) { current_velocities_[i] = msg->data; });

            camera_subscribers_[i] = create_subscription<sensor_msgs::msg::Image>(
                "/" + ns + "/front_camera", qos_camera,
                [this, i](const sensor_msgs::msg::Image::ConstSharedPtr msg) { cameraCallback(msg, i); });

            ss_subscribers_[i] = create_subscription<sensor_msgs::msg::Image>(
                "/" + ns + "/front_camera_ss", qos_camera,
                [this, i](const sensor_msgs::msg::Image::ConstSharedPtr msg) { ssCallback(msg, i); });

            odom_subscribers_[i] = create_subscription<nav_msgs::msg::Odometry>(
                "/carla/" + ns + "/odometry", qos_odom,
                [this, i](const nav_msgs::msg::Odometry::SharedPtr msg) { odomCallback(msg, i); });
            odom_subscribers_alt_[i] = create_subscription<nav_msgs::msg::Odometry>(
                "/" + ns + "/odometry", qos_odom,
                [this, i](const nav_msgs::msg::Odometry::SharedPtr msg) { odomCallback(msg, i); });
            pose_subscribers_[i] = create_subscription<geometry_msgs::msg::PoseStamped>(
                "/" + ns + "/pose_from_carla", qos_pose,
                [this, i](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { poseCallback(msg, i); });

            pid_controllers_[i] = std::make_shared<PIDController>(0.8f, 0.01f, 0.2f);
        }

        setup_timer_ = create_wall_timer(100ms, [this]() {
            if (setup_done_) return;
            auto self = shared_from_this();
            for (int i = 0; i < 3; ++i) {
                const std::string ns = "truck" + std::to_string(i);
                distance_sensors_[i] = std::make_shared<DistanceSensor>(self, ns);
                platooning_managers_[i] = std::make_shared<PlatooningManager>(self, ns);
            }
            setup_done_ = true;
        });

        change_lane_service_ = create_service<std_srvs::srv::SetBool>(
            "change_lane", std::bind(&LaneFollowingNode::changeLaneCallback, this, _1, _2));
        change_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(lc_dt_)),
            std::bind(&LaneFollowingNode::processChangeQueue, this));
        control_timer_ = create_wall_timer(50ms, std::bind(&LaneFollowingNode::publishCommandsFromModule, this));
        maneuver_timer_ = create_wall_timer(200ms, std::bind(&LaneFollowingNode::manageReorderManeuver, this));
    }

    void enqueueCommand(const PendingCommand &command) {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        change_queue_.push(command);
    }

    std::array<int, 3> truckOrder() const {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return truck_order_;
    }

    std::array<cv::Mat, 3> truckViews() const {
        std::lock_guard<std::mutex> lock(view_mutex_);
        return truck_views_;
    }

private:
    static const char *stateName(ManeuverState state) {
        switch (state) {
            case ManeuverState::IDLE: return "IDLE";
            case ManeuverState::LEADER_EXITS_LANE: return "LEADER_EXITS_LANE";
            case ManeuverState::LEADER_CREATES_GAP: return "LEADER_CREATES_GAP";
            case ManeuverState::SUCCESSOR_ENTERS_GAP: return "SUCCESSOR_ENTERS_GAP";
            case ManeuverState::FOLLOWER_ENTERS_GAP: return "FOLLOWER_ENTERS_GAP";
            case ManeuverState::REORDER_COMPLETE: return "REORDER_COMPLETE";
            case ManeuverState::COOLDOWN: return "COOLDOWN";
            case ManeuverState::LEADER_REENTERS_LANE: return "LEADER_REENTERS_LANE";
            case ManeuverState::PROMOTE_TARGET_EXITS: return "PROMOTE_TARGET_EXITS";
            case ManeuverState::PROMOTE_PLATOON_CREATES_GAP: return "PROMOTE_PLATOON_CREATES_GAP";
            case ManeuverState::PROMOTE_TARGET_REENTERS: return "PROMOTE_TARGET_REENTERS";
        }
        return "UNKNOWN";
    }

    void setManeuverState(ManeuverState next, const std::string &reason) {
        if (maneuver_state_ == next) return;
        RCLCPP_INFO(
            get_logger(),
            "[FSM] %s -> %s (%s)",
            stateName(maneuver_state_),
            stateName(next),
            reason.c_str());
        maneuver_state_ = next;
    }

    cv::Mat decodeColorImage(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
        try {
            return cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (const cv_bridge::Exception &) {
            cv::Mat frame = cv_bridge::toCvCopy(msg, msg->encoding)->image;
            if (frame.empty()) return frame;
            if (frame.channels() == 4) {
                cv::cvtColor(frame, frame, cv::COLOR_BGRA2BGR);
            } else if (frame.channels() == 3 && msg->encoding == "rgb8") {
                cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
            } else if (frame.channels() == 1) {
                cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);
            }
            return frame;
        }
    }

    cv::Mat decodeMonoImage(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
        try {
            return cv_bridge::toCvCopy(msg, "mono8")->image;
        } catch (const cv_bridge::Exception &) {
            cv::Mat frame = cv_bridge::toCvCopy(msg, msg->encoding)->image;
            if (frame.empty()) return frame;
            if (frame.channels() == 4) {
                cv::cvtColor(frame, frame, cv::COLOR_BGRA2GRAY);
            } else if (frame.channels() == 3 && msg->encoding == "rgb8") {
                cv::cvtColor(frame, frame, cv::COLOR_RGB2GRAY);
            } else if (frame.channels() == 3) {
                cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
            }
            return frame;
        }
    }

    void ssCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg, int truck_id) {
        try {
            cv::Mat ss_img = decodeMonoImage(msg);
            if (ss_img.empty()) return;
            cv::Mat lane_mask;
            cv::compare(ss_img, 24, lane_mask, cv::CMP_EQ);
            cv::morphologyEx(lane_mask, lane_mask, cv::MORPH_CLOSE, cv::Mat::ones(3, 3, CV_8U), cv::Point(-1, -1), 1);
            cv::Mat mask_bgr;
            cv::cvtColor(lane_mask, mask_bgr, cv::COLOR_GRAY2BGR);
            cv::Mat mask_bev = applyBirdsEyeView(mask_bgr);
            cv::cvtColor(mask_bev, ss_masks_bev_[truck_id], cv::COLOR_BGR2GRAY);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(), "SS callback error (truck %d): %s", truck_id, e.what());
        }
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg, int truck_id) {
        const bool was_valid = vehicle_poses_[truck_id].valid;
        const auto &position = msg->pose.pose.position;
        const auto &q = msg->pose.pose.orientation;
        const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        vehicle_poses_[truck_id] = VehiclePose{true, position.x, position.y, std::atan2(siny_cosp, cosy_cosp)};
        if (!was_valid) {
            RCLCPP_INFO(get_logger(), "Truck %d odometry 수신 시작", truck_id);
        }
    }

    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg, int truck_id) {
        const bool was_valid = vehicle_poses_[truck_id].valid;
        const auto &position = msg->pose.position;
        const auto &q = msg->pose.orientation;
        const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        vehicle_poses_[truck_id] = VehiclePose{true, position.x, position.y, std::atan2(siny_cosp, cosy_cosp)};
        if (!was_valid) {
            RCLCPP_INFO(get_logger(), "Truck %d pose_from_carla 수신 시작", truck_id);
        }
    }

    void cameraCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg, int truck_id) {
        try {
            cv::Mat frame = decodeColorImage(msg);
            if (frame.empty()) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Camera frame is empty for truck %d", truck_id);
                return;
            }
            cv::Mat bev_image = applyBirdsEyeView(frame);
            auto [lane_image, lane_positions] = detectLane(bev_image, truck_id, ss_masks_bev_[truck_id]);
            const bool is_changing = current_target_lane_[truck_id] != "center";
            const float steering_angle = calculateSteering(
                lane_positions,
                frame.cols,
                [this, truck_id](float error) { return pid_controllers_[truck_id]->compute(error); },
                current_target_lane_[truck_id],
                transition_factor_[truck_id],
                is_changing);

            std_msgs::msg::Float32 steer_msg;
            steer_msg.data = steering_angle;
            steer_publishers_[truck_id]->publish(steer_msg);

            last_steering_[truck_id] = steering_angle;
            if (truck_id == truck_order_[0]) leader_steering_ = steering_angle;
            {
                std::lock_guard<std::mutex> lock(view_mutex_);
                truck_views_[truck_id] = lane_image;
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(), "Camera callback error (truck %d): %s", truck_id, e.what());
            std::lock_guard<std::mutex> lock(view_mutex_);
            truck_views_[truck_id] = cv::Mat();
        }
    }

    void processChangeQueue() {
        std::queue<PendingCommand> local_queue;
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            std::swap(local_queue, change_queue_);
        }

        while (!local_queue.empty()) {
            const auto item = local_queue.front();
            local_queue.pop();
            if (item.type == PendingCommand::Type::Reorder) {
                RCLCPP_INFO(get_logger(), "[Keyboard] 순서 재배치 요청 (방향: %s)", item.direction.c_str());
                startReorderManeuver(item.direction);
            } else if (item.type == PendingCommand::Type::Promote) {
                RCLCPP_INFO(get_logger(), "[Keyboard] Promote 요청 (타겟: %d, 방향: %s)", item.truck_id, item.direction.c_str());
                startPromoteManeuver(item.truck_id, item.direction);
            } else if (maneuver_state_ == ManeuverState::IDLE) {
                RCLCPP_INFO(get_logger(), "[Keyboard] Truck %d '%s' 차선 변경 요청", item.truck_id, item.direction.c_str());
                guardedLaneChange(item.truck_id, item.direction, "MANUAL");
            } else {
                RCLCPP_WARN(get_logger(), "기동 중에는 개별 차선 변경을 허용하지 않습니다.");
            }
        }
    }

    void changeLaneCallback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        const std::string direction = request->data ? "left" : "right";
        if (maneuver_state_ != ManeuverState::IDLE) {
            response->success = false;
            response->message = "재배치 중에는 서비스로 개별 변경 불가";
            return;
        }

        RCLCPP_INFO(get_logger(), "서비스 호출: 리더부터 %s 방향 순차 차선 변경", direction.c_str());
        if (guardedLaneChange(truck_order_[0], direction, "MANUAL")) {
            for (int i = 1; i < 3; ++i) {
                const int follower_id = truck_order_[i];
                auto holder = std::make_shared<rclcpp::TimerBase::SharedPtr>();
                *holder = create_wall_timer(std::chrono::milliseconds(i * 2000), [this, follower_id, direction, holder]() {
                    guardedLaneChange(follower_id, direction, "MANUAL");
                    if (*holder) (*holder)->cancel();
                });
                deferred_timers_.push_back(*holder);
            }
            response->success = true;
            response->message = direction + " 차선 변경 절차 예약됨";
        } else {
            response->success = false;
            response->message = "차선 변경 실패";
        }
    }

    bool changeLane(int truck_id, const std::string &direction) {
        if (current_target_lane_[truck_id] != "center") {
            RCLCPP_WARN(get_logger(), "Truck %d: 이미 차선 변경 중", truck_id);
            return false;
        }

        RCLCPP_INFO(get_logger(), "Truck %d: %s으로 차선 변경 시작", truck_id, direction.c_str());
        current_target_lane_[truck_id] = direction;
        transition_factor_[truck_id] = 0.0f;
        pid_controllers_[truck_id]->reset();

        if (transition_timers_[truck_id]) transition_timers_[truck_id]->cancel();
        transition_timers_[truck_id] = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(lc_dt_)),
            [this, truck_id, direction]() {
            if (current_target_lane_[truck_id] != direction) {
                transition_timers_[truck_id]->cancel();
                return;
            }
            transition_factor_[truck_id] += lc_step_;
            if (transition_factor_[truck_id] >= 1.0f) {
                transition_factor_[truck_id] = 1.0f;
                RCLCPP_INFO(get_logger(), "Truck %d: 차선 변경 애니메이션 완료", truck_id);
                transition_timers_[truck_id]->cancel();
            }
        });
        return true;
    }

    void resetLaneState(int truck_id) {
        RCLCPP_INFO(get_logger(), "Truck %d: 중앙 차선 복귀 상태로 리셋", truck_id);
        if (transition_timers_[truck_id]) transition_timers_[truck_id]->cancel();
        current_target_lane_[truck_id] = "center";
        transition_factor_[truck_id] = 1.0f;
        pid_controllers_[truck_id]->reset();
    }

    bool guardedLaneChange(int truck_id, const std::string &direction, const std::string &tag) {
        if (guard_pending_[truck_id]) {
            RCLCPP_INFO(get_logger(), "[%s] Truck %d 가드 이미 진행 중", tag.c_str(), truck_id);
            return false;
        }
        guard_pending_[truck_id] = true;
        guard_start_time_[truck_id] = now();

        guard_timers_[truck_id] = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(change_check_period_)),
            [this, truck_id, direction, tag]() {
            if (tag != "MANUAL" &&
                (maneuver_state_ == ManeuverState::IDLE || maneuver_state_ == ManeuverState::REORDER_COMPLETE || maneuver_state_ == ManeuverState::COOLDOWN)) {
                RCLCPP_INFO(get_logger(), "[%s] Truck %d 가드 취소(FSM 종료/전이)", tag.c_str(), truck_id);
                cancelGuard(truck_id);
                return;
            }

            const double elapsed = (now() - guard_start_time_[truck_id]).seconds();
            if (elapsed > change_timeout_sec_) {
                RCLCPP_WARN(get_logger(), "[%s] Truck %d 타임아웃(%.1fs). 취소.", tag.c_str(), truck_id, change_timeout_sec_);
                cancelGuard(truck_id);
                return;
            }

            const float relax_ratio = std::min(1.0f, static_cast<float>(elapsed) / std::max(1e-3f, relax_full_time_));
            const float relax_gain = relax_full_gain_ * relax_ratio;
            const float scale = guardScale(tag);
            const float dyn_min = std::max(relax_floor_, change_min_dist_ - relax_gain) * scale;
            if (!distance_sensors_[truck_id]) {
                cancelGuard(truck_id);
                return;
            }
            const auto dist = distance_sensors_[truck_id]->get_distance();
            if (!dist || *dist > clear_start_dist_ || *dist >= dyn_min) {
                if (!dist || *dist > clear_start_dist_) {
                    const std::string dist_str = dist ? std::to_string(*dist) : "None";
                    RCLCPP_INFO(
                        get_logger(),
                        "[%s] Truck %d FAST-START: dist=%s >= clear=%.1fm",
                        tag.c_str(),
                        truck_id,
                        dist_str.c_str(),
                        clear_start_dist_);
                } else {
                    RCLCPP_INFO(
                        get_logger(),
                        "[%s] Truck %d OK: dist=%.1fm >= dyn_min=%.1fm -> 차선 변경 시작",
                        tag.c_str(),
                        truck_id,
                        *dist,
                        dyn_min);
                }
                cancelGuard(truck_id);
                changeLane(truck_id, direction);
            } else {
                RCLCPP_INFO(
                    get_logger(),
                    "[%s] Truck %d 대기: dist=%.1fm (< %.1fm). 재확인 예정",
                    tag.c_str(),
                    truck_id,
                    *dist,
                    dyn_min);
            }
        });
        return true;
    }

    void cancelGuard(int truck_id) {
        if (guard_timers_[truck_id]) {
            guard_timers_[truck_id]->cancel();
            guard_timers_[truck_id].reset();
        }
        guard_pending_[truck_id] = false;
    }

    bool startReorderManeuver(const std::string &direction) {
        if (maneuver_state_ != ManeuverState::IDLE) {
            RCLCPP_WARN(get_logger(), "이미 재배치 진행 중");
            return false;
        }
        reorder_direction_ = direction;
        exiting_leader_id_ = truck_order_[0];
        successor_id_ = truck_order_[1];
        follower_id_ = truck_order_[2];
        setManeuverState(ManeuverState::LEADER_EXITS_LANE, "reorder start");
        RCLCPP_INFO(get_logger(), "= 재배치 시작(%s) : 리더 %d 차선이탈 =", direction.c_str(), exiting_leader_id_);
        return changeLane(exiting_leader_id_, direction);
    }

    bool startPromoteManeuver(int target_id, const std::string &direction) {
        if (maneuver_state_ != ManeuverState::IDLE) {
            RCLCPP_WARN(get_logger(), "이미 다른 기동 진행 중");
            return false;
        }
        if (target_id != truck_order_[1] && target_id != truck_order_[2]) {
            RCLCPP_WARN(get_logger(), "Truck %d는 후행 차량이 아니므로 선두로 보낼 수 없습니다.", target_id);
            return false;
        }
        reorder_direction_ = direction;
        promote_target_id_ = target_id;
        promote_original_leader_id_ = truck_order_[0];
        setManeuverState(ManeuverState::PROMOTE_TARGET_EXITS, "promote start");
        RCLCPP_INFO(get_logger(), "= Promote 시작(%s) : 타겟 차량 %d 차선 이탈 =", direction.c_str(), promote_target_id_);
        return changeLane(promote_target_id_, direction);
    }

    bool laneChangeComplete(int truck_id) const { return transition_factor_[truck_id] >= 1.0f; }

    std::optional<double> forwardDistanceFrom(int base_id, int other_id) const {
        if (!vehicle_poses_[base_id].valid || !vehicle_poses_[other_id].valid) return std::nullopt;
        const auto &base = vehicle_poses_[base_id];
        const auto &other = vehicle_poses_[other_id];
        const double dx = other.x - base.x;
        const double dy = other.y - base.y;
        return dx * std::cos(base.yaw) + dy * std::sin(base.yaw);
    }

    std::optional<double> lateralDistanceFrom(int base_id, int other_id) const {
        if (!vehicle_poses_[base_id].valid || !vehicle_poses_[other_id].valid) return std::nullopt;
        const auto &base = vehicle_poses_[base_id];
        const auto &other = vehicle_poses_[other_id];
        const double dx = other.x - base.x;
        const double dy = other.y - base.y;
        return std::abs(-dx * std::sin(base.yaw) + dy * std::cos(base.yaw));
    }

    void manageReorderManeuver() {
        if (maneuver_state_ == ManeuverState::IDLE) return;

        if (maneuver_state_ == ManeuverState::LEADER_EXITS_LANE && laneChangeComplete(exiting_leader_id_)) {
            resetLaneState(exiting_leader_id_);
            setManeuverState(ManeuverState::LEADER_CREATES_GAP, "leader exited lane");
            RCLCPP_INFO(get_logger(), "리더 갭 생성 단계 진입(감속 및 거리 확인 시작)");
            return;
        }

        if (maneuver_state_ == ManeuverState::LEADER_CREATES_GAP) {
            const auto gap = forwardDistanceFrom(successor_id_, exiting_leader_id_);
            if (gap && *gap < 0.0 && std::abs(*gap) >= reorder_safe_gap_distance_m_) {
                RCLCPP_INFO(get_logger(), "안전거리 확보 (뒤로 %.1fm). 후속 차량 진입 시작.", std::abs(*gap));
                setManeuverState(ManeuverState::SUCCESSOR_ENTERS_GAP, "gap created");
                guardedLaneChange(successor_id_, reorder_direction_, "SUCCESSOR");
            } else if (gap) {
                RCLCPP_INFO_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    1000,
                    "갭 생성 대기 중... 리더 상대 위치: %.1fm (목표: 뒤로 %.1fm 이상)",
                    *gap,
                    reorder_safe_gap_distance_m_);
            } else {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    2000,
                    "Reorder: 차량 transform 정보를 기다리는 중...");
            }
            return;
        }

        if (maneuver_state_ == ManeuverState::SUCCESSOR_ENTERS_GAP && laneChangeComplete(successor_id_)) {
            resetLaneState(successor_id_);
            setManeuverState(ManeuverState::FOLLOWER_ENTERS_GAP, "successor entered");
            RCLCPP_INFO(get_logger(), "후속 차량(2번) 차선 변경 시도(라이다 가드)");
            guardedLaneChange(follower_id_, reorder_direction_, "FOLLOWER");
            return;
        }

        if (maneuver_state_ == ManeuverState::FOLLOWER_ENTERS_GAP && laneChangeComplete(follower_id_)) {
            resetLaneState(follower_id_);
            setManeuverState(ManeuverState::LEADER_REENTERS_LANE, "follower entered");
            RCLCPP_INFO(get_logger(), "리더 재합류 대기 시작");
            startRejoinCheck();
            return;
        }

        if (maneuver_state_ == ManeuverState::LEADER_REENTERS_LANE) {
            if (laneChangeComplete(exiting_leader_id_)) {
                setManeuverState(ManeuverState::REORDER_COMPLETE, "leader reentered");
                finalizeReorder();
            }
            return;
        }

        if (maneuver_state_ == ManeuverState::PROMOTE_TARGET_EXITS && laneChangeComplete(promote_target_id_)) {
            resetLaneState(promote_target_id_);
            setManeuverState(ManeuverState::PROMOTE_PLATOON_CREATES_GAP, "promote target exited");
            RCLCPP_INFO(get_logger(), "Promote: 대상 차량 이탈 완료. 군집 감속 및 추월 대기.");
            return;
        }

        if (maneuver_state_ == ManeuverState::PROMOTE_PLATOON_CREATES_GAP) {
            const auto dist = forwardDistanceFrom(promote_original_leader_id_, promote_target_id_);
            if (dist && *dist > 0.0 && *dist >= 10.0) {
                RCLCPP_INFO(get_logger(), "Promote: 안전거리(%.1fm) 확보. 대상 차량 선두 합류 시도.", *dist);
                setManeuverState(ManeuverState::PROMOTE_TARGET_REENTERS, "promote safe distance");
                changeLane(promote_target_id_, opposite(reorder_direction_));
            }
            return;
        }

        if (maneuver_state_ == ManeuverState::PROMOTE_TARGET_REENTERS && laneChangeComplete(promote_target_id_)) {
            setManeuverState(ManeuverState::REORDER_COMPLETE, "promote target reentered");
            finalizeReorder();
        }
    }

    void startRejoinCheck() {
        if (rejoin_timer_) rejoin_timer_->cancel();
        rejoin_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(rejoin_check_period_)),
            [this]() {
            if (maneuver_state_ != ManeuverState::LEADER_REENTERS_LANE) {
                rejoin_timer_->cancel();
                return;
            }

            const auto forward = forwardDistanceFrom(follower_id_, exiting_leader_id_);
            const auto lateral = lateralDistanceFrom(follower_id_, exiting_leader_id_);
            if (!forward || !lateral) return;

            if (*forward < 0.0 && std::abs(*forward) >= 10.0 && std::abs(*forward) <= 18.0 && *lateral <= 1.0) {
                RCLCPP_INFO(
                    get_logger(),
                    "[REJOIN by Coords] 조건 만족 (전후방: %.1fm, 좌우: %.1fm). 리더 %d 재합류 시작.",
                    *forward,
                    *lateral,
                    exiting_leader_id_);
                guardedLaneChange(exiting_leader_id_, opposite(reorder_direction_), "REJOIN");
                rejoin_timer_->cancel();
            } else {
                RCLCPP_DEBUG(
                    get_logger(),
                    "[REJOIN by Coords] 재합류 대기... (전후방: %.1fm, 좌우: %.1fm)",
                    *forward,
                    *lateral);
            }
        });
    }

    void finalizeReorder() {
        RCLCPP_INFO(get_logger(), "===== 기동 완료: 모든 상태 초기화 =====");
        for (int i = 0; i < 3; ++i) {
            cancelGuard(i);
            resetLaneState(i);
            if (platooning_managers_[i]) {
                platooning_managers_[i]->integral_ = 0.0f;
                platooning_managers_[i]->prev_error_ = 0.0f;
                platooning_managers_[i]->prev_time_ = now();
            }
        }

        if (rejoin_timer_) rejoin_timer_->cancel();

        if (exiting_leader_id_ != -1) {
            std::lock_guard<std::mutex> lock(state_mutex_);
            truck_order_ = {successor_id_, follower_id_, exiting_leader_id_};
            RCLCPP_INFO(get_logger(), "= Reorder 완료: 새로운 순서는 [%d, %d, %d] =", truck_order_[0], truck_order_[1], truck_order_[2]);
        } else if (promote_target_id_ != -1) {
            std::array<int, 3> reordered = {promote_target_id_, -1, -1};
            int idx = 1;
            for (int id : truck_order_) {
                if (id == promote_target_id_) continue;
                reordered[idx++] = id;
            }
            std::lock_guard<std::mutex> lock(state_mutex_);
            truck_order_ = reordered;
            RCLCPP_INFO(get_logger(), "= Promote 완료: 새로운 순서는 [%d, %d, %d] =", truck_order_[0], truck_order_[1], truck_order_[2]);
        }

        setManeuverState(ManeuverState::COOLDOWN, "finalized");
        auto holder = std::make_shared<rclcpp::TimerBase::SharedPtr>();
        *holder = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(cooldown_sec_)),
            [this, holder]() {
            resetManeuverVariables();
            if (*holder) (*holder)->cancel();
        });
        deferred_timers_.push_back(*holder);
    }

    void resetManeuverVariables() {
        setManeuverState(ManeuverState::IDLE, "cooldown done");
        exiting_leader_id_ = -1;
        successor_id_ = -1;
        follower_id_ = -1;
        promote_target_id_ = -1;
        promote_original_leader_id_ = -1;
        RCLCPP_INFO(get_logger(), "===== 쿨다운 종료: IDLE 상태로 복귀 =====");
    }

    std::string opposite(const std::string &direction) const {
        return direction == "left" ? "right" : "left";
    }

    void publishCommandsFromModule() {
        if (!setup_done_) return;

        const int current_leader_id = truck_order_[0];
        const bool is_promoting =
            maneuver_state_ == ManeuverState::PROMOTE_TARGET_EXITS ||
            maneuver_state_ == ManeuverState::PROMOTE_PLATOON_CREATES_GAP ||
            maneuver_state_ == ManeuverState::PROMOTE_TARGET_REENTERS;

        const auto leader_front_distance = distance_sensors_[current_leader_id]->get_distance();
        emergency_stop_ = leader_front_distance && *leader_front_distance < leader_emergency_distance_;

        for (int i = 0; i < 3; ++i) {
            const int truck_id = truck_order_[i];
            if (emergency_stop_) {
                std_msgs::msg::Float32 msg;
                msg.data = -1.0f;
                throttle_publishers_[truck_id]->publish(msg);
                continue;
            }

            if (is_promoting) {
                if (truck_id == promote_target_id_) {
                    const float speed = i == 1 ? target_velocity_ * 1.2f : (i == 2 ? target_velocity_ * 1.1f : target_velocity_ * 1.2f);
                    publish_commands({throttle_publishers_[truck_id]}, {current_velocities_[truck_id]}, speed, {last_steering_[truck_id]});
                    continue;
                }
                if (promote_target_id_ == truck_order_[1] && truck_id == truck_order_[2]) {
                    publish_commands({throttle_publishers_[truck_id]}, {current_velocities_[truck_id]}, target_velocity_, {last_steering_[truck_id]});
                    continue;
                }
                if (maneuver_state_ == ManeuverState::PROMOTE_PLATOON_CREATES_GAP || maneuver_state_ == ManeuverState::PROMOTE_TARGET_REENTERS) {
                    publish_commands({throttle_publishers_[truck_id]}, {current_velocities_[truck_id]}, target_velocity_ * 0.8f, {last_steering_[truck_id]});
                    continue;
                }
            }

            if (i == 0) {
                float final_target_velocity = target_velocity_;
                if (leader_front_distance) {
                    if (*leader_front_distance <= 10.0f) {
                        final_target_velocity = 0.0f;
                    } else if (*leader_front_distance < 30.0f) {
                        final_target_velocity = std::max(0.0f, target_velocity_ * ((*leader_front_distance - 10.0f) / 20.0f));
                    }
                }
                if (truck_id == exiting_leader_id_ &&
                    (maneuver_state_ == ManeuverState::LEADER_CREATES_GAP ||
                     maneuver_state_ == ManeuverState::SUCCESSOR_ENTERS_GAP ||
                     maneuver_state_ == ManeuverState::FOLLOWER_ENTERS_GAP)) {
                    final_target_velocity = std::min(final_target_velocity, target_velocity_ * leader_slow_factor_);
                }

                publish_commands({throttle_publishers_[truck_id]}, {current_velocities_[truck_id]}, final_target_velocity, {last_steering_[truck_id]});
            } else {
                const auto lidar_distance = distance_sensors_[truck_id]->get_distance();
                if (!lidar_distance || *lidar_distance > loss_dist_) {
                    publish_commands({throttle_publishers_[truck_id]}, {current_velocities_[truck_id]}, target_velocity_, {last_steering_[truck_id]});
                } else {
                    platooning_managers_[truck_id]->update_distance(*lidar_distance, emergency_stop_);
                }
            }
        }
    }

    float guardScale(const std::string &tag) const {
        if (tag == "SUCCESSOR" || tag == "FOLLOWER") return 0.9f;
        if (tag == "PROMOTE_TARGET" || tag == "REJOIN" || tag.empty()) return 1.0f;
        return 1.0f;
    }

private:
    bool emergency_stop_ = false;
    bool setup_done_ = false;
    float leader_steering_ = 0.0f;
    std::array<int, 3> truck_order_;
    std::array<std::string, 3> current_target_lane_;
    std::array<float, 3> transition_factor_;
    std::array<float, 3> current_velocities_;
    std::array<float, 3> last_steering_;
    std::array<cv::Mat, 3> truck_views_;
    std::array<cv::Mat, 3> ss_masks_bev_;
    std::array<VehiclePose, 3> vehicle_poses_;

    ManeuverState maneuver_state_;
    std::string reorder_direction_;
    int exiting_leader_id_ = -1;
    int successor_id_ = -1;
    int follower_id_ = -1;
    int promote_target_id_ = -1;
    int promote_original_leader_id_ = -1;

    std::mutex queue_mutex_;
    mutable std::mutex view_mutex_;
    mutable std::mutex state_mutex_;
    std::queue<PendingCommand> change_queue_;

    std::array<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr, 3> steer_publishers_;
    std::array<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr, 3> throttle_publishers_;
    std::array<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr, 3> camera_subscribers_;
    std::array<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr, 3> ss_subscribers_;
    std::array<rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr, 3> velocity_subscribers_;
    std::array<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr, 3> odom_subscribers_;
    std::array<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr, 3> odom_subscribers_alt_;
    std::array<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr, 3> pose_subscribers_;
    std::array<std::shared_ptr<PIDController>, 3> pid_controllers_;
    std::array<std::shared_ptr<DistanceSensor>, 3> distance_sensors_;
    std::array<std::shared_ptr<PlatooningManager>, 3> platooning_managers_;
    std::array<rclcpp::TimerBase::SharedPtr, 3> transition_timers_;
    std::array<rclcpp::TimerBase::SharedPtr, 3> guard_timers_;
    std::array<bool, 3> guard_pending_{false, false, false};
    std::array<rclcpp::Time, 3> guard_start_time_{now(), now(), now()};

    float target_velocity_ = 14.5f;
    float lc_dt_ = 0.1f;
    float lc_step_ = 0.03f;
    float cooldown_sec_ = 1.0f;
    float leader_slow_factor_ = 0.65f;
    float loss_dist_ = 40.0f;
    float leader_emergency_distance_ = 3.0f;
    float change_min_dist_ = 20.0f;
    float change_check_period_ = 0.05f;
    float clear_start_dist_ = 25.0f;
    float relax_full_gain_ = 8.0f;
    float relax_full_time_ = 1.2f;
    float relax_floor_ = 18.0f;
    float change_timeout_sec_ = 5.0f;
    float rejoin_check_period_ = 0.05f;
    float reorder_safe_gap_distance_m_ = 25.0f;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr change_lane_service_;
    rclcpp::TimerBase::SharedPtr setup_timer_;
    rclcpp::TimerBase::SharedPtr change_timer_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr maneuver_timer_;
    rclcpp::TimerBase::SharedPtr rejoin_timer_;
    std::vector<rclcpp::TimerBase::SharedPtr> deferred_timers_;
};

static void opencvLoop(const std::shared_ptr<LaneFollowingNode> &node) {
    cv::startWindowThread();
    cv::namedWindow("Combined Bird-Eye View", cv::WINDOW_NORMAL);
    cv::resizeWindow("Combined Bird-Eye View", 1600, 600);

    while (rclcpp::ok()) {
        const auto order = node->truckOrder();
        const auto views = node->truckViews();
        std::vector<cv::Mat> panels;

        for (int truck_id : order) {
            if (views[truck_id].empty()) {
                panels.push_back(cv::Mat::zeros(480, 640, CV_8UC3));
            } else {
                panels.push_back(views[truck_id]);
            }
        }

        if (panels.size() == 3) {
            cv::Mat combined;
            cv::hconcat(panels, combined);
            cv::imshow("Combined Bird-Eye View", combined);
        }

        const int key = cv::waitKey(1) & 0xFF;
        if (key == 't') node->enqueueCommand({PendingCommand::Type::Reorder, -1, "left"});
        else if (key == 'y') node->enqueueCommand({PendingCommand::Type::Reorder, -1, "right"});
        else if (key == 'j') node->enqueueCommand({PendingCommand::Type::Promote, order[1], "left"});
        else if (key == 'k') node->enqueueCommand({PendingCommand::Type::Promote, order[2], "left"});
        else if (key == 'q') node->enqueueCommand({PendingCommand::Type::Manual, 0, "left"});
        else if (key == 'e') node->enqueueCommand({PendingCommand::Type::Manual, 0, "right"});
        else if (key == 'a') node->enqueueCommand({PendingCommand::Type::Manual, 1, "left"});
        else if (key == 'd') node->enqueueCommand({PendingCommand::Type::Manual, 1, "right"});
        else if (key == 'z') node->enqueueCommand({PendingCommand::Type::Manual, 2, "left"});
        else if (key == 'c') node->enqueueCommand({PendingCommand::Type::Manual, 2, "right"});
        else if (key == 27) break;
    }

    cv::destroyAllWindows();
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaneFollowingNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread ros_thread([&executor]() { executor.spin(); });

    opencvLoop(node);

    executor.cancel();
    if (ros_thread.joinable()) ros_thread.join();
    rclcpp::shutdown();
    return 0;
}
