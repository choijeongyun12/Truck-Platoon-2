#ifndef LANE_DETECT_HPP
#define LANE_DETECT_HPP

#include <functional>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

struct LanePositions {
    std::optional<int> center_left;
    std::optional<int> center_right;
    std::optional<int> adj_left;
    std::optional<int> adj_right;
    std::optional<int> center_left_mid;
    std::optional<int> center_right_mid;
    std::optional<int> adj_left_mid;
    std::optional<int> adj_right_mid;
};

inline cv::Mat applyBirdsEyeView(const cv::Mat &image) {
    const int height = image.rows;
    const int width = image.cols;

    cv::Point2f src[4] = {
        cv::Point2f(width * 0.3f, height * 0.4f),
        cv::Point2f(width * 0.7f, height * 0.4f),
        cv::Point2f(width, height),
        cv::Point2f(0, height),
    };

    cv::Point2f dst[4] = {
        cv::Point2f(0, 0),
        cv::Point2f(width, 0),
        cv::Point2f(width, height),
        cv::Point2f(0, height),
    };

    cv::Mat transform = cv::getPerspectiveTransform(src, dst);
    cv::Mat result;
    cv::warpPerspective(image, result, transform, cv::Size(width, height));
    return result;
}

inline std::vector<int> findHistogramPeaks(const std::vector<int> &histogram, int min_height, int min_distance) {
    std::vector<int> peaks;
    for (int i = 1; i < static_cast<int>(histogram.size()) - 1; ++i) {
        if (histogram[i] < min_height) continue;
        if (histogram[i] < histogram[i - 1] || histogram[i] < histogram[i + 1]) continue;
        if (!peaks.empty() && i - peaks.back() < min_distance) {
            if (histogram[i] > histogram[peaks.back()]) peaks.back() = i;
            continue;
        }
        peaks.push_back(i);
    }
    return peaks;
}

inline std::pair<std::optional<int>, std::optional<int>> getFallbackSeeds(const cv::Mat &edges) {
    const int h = edges.rows;
    const int w = edges.cols;
    const cv::Mat roi = edges.rowRange(static_cast<int>(h * 0.6), h);
    std::vector<int> histogram(w, 0);

    for (int y = 0; y < roi.rows; ++y) {
        for (int x = 0; x < roi.cols; ++x) {
            histogram[x] += roi.at<unsigned char>(y, x);
        }
    }

    const auto max_it = std::max_element(histogram.begin(), histogram.end());
    if (max_it == histogram.end() || *max_it <= 10) {
        return {std::nullopt, std::nullopt};
    }

    const int mid = w / 2;
    auto left_it = std::max_element(histogram.begin(), histogram.begin() + mid);
    auto right_it = std::max_element(histogram.begin() + mid, histogram.end());
    return {
        std::optional<int>(std::distance(histogram.begin(), left_it)),
        std::optional<int>(std::distance(histogram.begin(), right_it)),
    };
}

inline std::pair<cv::Mat, LanePositions> detectLane(const cv::Mat &image, int truck_id, const cv::Mat &ss_mask = cv::Mat()) {
    static std::unordered_map<int, LanePositions> last_known_positions;

    const int height = image.rows;
    const int width = image.cols;
    cv::Mat edges;

    if (!ss_mask.empty()) {
        if (ss_mask.size() == image.size()) {
            edges = ss_mask.clone();
        } else {
            cv::resize(ss_mask, edges, image.size(), 0, 0, cv::INTER_NEAREST);
        }
    } else {
        cv::Mat hls;
        cv::cvtColor(image, hls, cv::COLOR_BGR2HLS);
        cv::Mat white_mask;
        cv::inRange(hls, cv::Scalar(0, 200, 0), cv::Scalar(180, 255, 70), white_mask);
        cv::Mat masked_hls;
        cv::bitwise_and(image, image, masked_hls, white_mask);
        cv::Mat gray;
        cv::cvtColor(masked_hls, gray, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0);
        cv::Canny(gray, edges, 50, 150);
    }

    std::vector<int> histogram(width, 0);
    for (int y = height / 2; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            histogram[x] += edges.at<unsigned char>(y, x);
        }
    }

    const auto peaks = findHistogramPeaks(histogram, 60, width / 8);
    const int mid = width / 2;
    std::vector<int> left_lanes;
    std::vector<int> right_lanes;
    for (int peak : peaks) {
        if (peak < mid) left_lanes.push_back(peak);
        if (peak > mid) right_lanes.push_back(peak);
    }
    std::sort(left_lanes.begin(), left_lanes.end(), std::greater<int>());
    std::sort(right_lanes.begin(), right_lanes.end());

    LanePositions positions;
    if (!left_lanes.empty()) positions.center_left = left_lanes[0];
    if (!right_lanes.empty()) positions.center_right = right_lanes[0];
    if (left_lanes.size() > 1) positions.adj_left = left_lanes[1];
    if (right_lanes.size() > 1) positions.adj_right = right_lanes[1];

    const auto last_it = last_known_positions.find(truck_id);
    if (last_it != last_known_positions.end()) {
        if (!positions.center_left) positions.center_left = last_it->second.center_left;
        if (!positions.center_right) positions.center_right = last_it->second.center_right;
        if (!positions.adj_left) positions.adj_left = last_it->second.adj_left;
        if (!positions.adj_right) positions.adj_right = last_it->second.adj_right;
    }

    if (!positions.center_left || !positions.center_right) {
        auto [fallback_left, fallback_right] = getFallbackSeeds(edges);
        if (!positions.center_left) positions.center_left = fallback_left;
        if (!positions.center_right) positions.center_right = fallback_right;
    }
    if (!positions.center_left) positions.center_left = static_cast<int>(width * 0.25f);
    if (!positions.center_right) positions.center_right = static_cast<int>(width * 0.75f);

    const int nwindows = 10;
    const int margin = 80;
    const int minpix = 5;
    const int window_height = height / nwindows;

    std::vector<cv::Point> nonzero;
    cv::findNonZero(edges, nonzero);
    cv::Mat window_img = cv::Mat::zeros(image.size(), image.type());

    std::vector<std::pair<std::string, int>> tracked = {
        {"center_left", *positions.center_left},
        {"center_right", *positions.center_right},
    };
    if (positions.adj_left) tracked.emplace_back("adj_left", *positions.adj_left);
    if (positions.adj_right) tracked.emplace_back("adj_right", *positions.adj_right);
    std::unordered_map<std::string, std::vector<int>> lane_paths;
    for (const auto &lane : tracked) lane_paths[lane.first] = {};

    for (int window = nwindows - 1; window >= 0; --window) {
        const int win_y_low = window * window_height;
        const int win_y_high = (window + 1) * window_height;

        for (auto &lane : tracked) {
            const int current_x = lane.second;
            const int win_x_low = current_x - margin;
            const int win_x_high = current_x + margin;

            cv::Scalar color(255, 255, 255);
            if (lane.first == "adj_right") color = cv::Scalar(0, 255, 0);
            if (lane.first == "center_left") color = cv::Scalar(255, 0, 0);
            if (lane.first == "center_right") color = cv::Scalar(0, 0, 255);
            cv::rectangle(window_img, cv::Point(win_x_low, win_y_low), cv::Point(win_x_high, win_y_high), color, 2);

            std::vector<int> indices;
            for (size_t i = 0; i < nonzero.size(); ++i) {
                const auto &p = nonzero[i];
                if (p.y >= win_y_low && p.y < win_y_high && p.x >= win_x_low && p.x < win_x_high) {
                    indices.push_back(static_cast<int>(i));
                }
            }
            if (static_cast<int>(indices.size()) > minpix) {
                float mean_x = 0.0f;
                for (int idx : indices) mean_x += nonzero[idx].x;
                mean_x /= static_cast<float>(indices.size());
                lane.second = static_cast<int>(indices.size() < static_cast<size_t>(minpix * 3) ? (0.7f * current_x + 0.3f * mean_x) : mean_x);
            }
            lane_paths[lane.first].push_back(lane.second);
        }
    }

    for (const auto &lane : tracked) {
        if (lane.first == "center_left") positions.center_left = lane.second;
        if (lane.first == "center_right") positions.center_right = lane.second;
        if (lane.first == "adj_left") positions.adj_left = lane.second;
        if (lane.first == "adj_right") positions.adj_right = lane.second;
    }

    auto set_mid = [&lane_paths, &positions](const std::string &key) {
        auto it = lane_paths.find(key);
        if (it == lane_paths.end() || it->second.empty()) return;
        const auto &path = it->second;
        const int mid_x = path[path.size() / 2];
        if (key == "center_left") positions.center_left_mid = mid_x;
        if (key == "center_right") positions.center_right_mid = mid_x;
        if (key == "adj_left") positions.adj_left_mid = mid_x;
        if (key == "adj_right") positions.adj_right_mid = mid_x;
    };
    set_mid("center_left");
    set_mid("center_right");
    set_mid("adj_left");
    set_mid("adj_right");

    last_known_positions[truck_id] = positions;

    cv::Mat lane_overlay;
    cv::addWeighted(image, 1.0, window_img, 0.3, 0.0, lane_overlay);
    cv::rectangle(lane_overlay, cv::Point(0, 0), cv::Point(width - 1, height - 1), cv::Scalar(0, 0, 255), 2);
    cv::putText(lane_overlay, std::to_string(truck_id), cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 0), 3, cv::LINE_AA);
    cv::putText(lane_overlay, std::to_string(truck_id), cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

    return {lane_overlay, positions};
}

inline float calculateSteering(
    const LanePositions &lane_positions,
    int img_width,
    std::function<float(float)> pid_compute,
    const std::string &target_lane = "center",
    float transition_factor = 0.0f,
    bool is_lane_changing = false) {
    if (!lane_positions.center_left || !lane_positions.center_right) {
        return 0.0f;
    }

    auto pick_x = [&lane_positions, is_lane_changing](const std::string &key) -> std::optional<int> {
        if (!is_lane_changing) {
            if (key == "center_left" && lane_positions.center_left_mid) return lane_positions.center_left_mid;
            if (key == "center_right" && lane_positions.center_right_mid) return lane_positions.center_right_mid;
            if (key == "adj_left" && lane_positions.adj_left_mid) return lane_positions.adj_left_mid;
            if (key == "adj_right" && lane_positions.adj_right_mid) return lane_positions.adj_right_mid;
        }
        if (key == "center_left") return lane_positions.center_left;
        if (key == "center_right") return lane_positions.center_right;
        if (key == "adj_left") return lane_positions.adj_left;
        if (key == "adj_right") return lane_positions.adj_right;
        return std::nullopt;
    };

    const float img_center = img_width / 2.0f;
    const auto current_left_opt = pick_x("center_left");
    const auto current_right_opt = pick_x("center_right");
    if (!current_left_opt || !current_right_opt) {
        return 0.0f;
    }
    const float current_left = static_cast<float>(*current_left_opt);
    const float current_right = static_cast<float>(*current_right_opt);
    const float lane_width = current_right - current_left;

    float target_left = current_left;
    float target_right = current_right;
    if (target_lane == "left") {
        const auto adj_left = pick_x("adj_left");
        if (adj_left) {
            target_left = static_cast<float>(*adj_left);
            target_right = target_left + lane_width;
        } else {
            target_left = current_left - lane_width;
            target_right = current_right - lane_width;
        }
    } else if (target_lane == "right") {
        const auto adj_right = pick_x("adj_right");
        if (adj_right) {
            target_right = static_cast<float>(*adj_right);
            target_left = target_right - lane_width;
        } else {
            target_left = current_left + lane_width;
            target_right = current_right + lane_width;
        }
    }

    if (is_lane_changing) {
        const float tf = std::clamp(transition_factor, 0.0f, 1.0f);
        transition_factor = tf * tf * (3.0f - 2.0f * tf);  // smoothstep
    }

    const float blended_left = current_left + (target_left - current_left) * transition_factor;
    const float blended_right = current_right + (target_right - current_right) * transition_factor;
    const float lane_center = (blended_left + blended_right) / 2.0f;
    float error = (lane_center - img_center) / img_center;

    if (is_lane_changing) {
        // Compress large errors during lane-change to suppress over-steering.
        error = std::tanh(error * 1.3f) * 0.75f;
    }
    const float pid_output = pid_compute(error);

    if (is_lane_changing) {
        return std::clamp(-pid_output * 12.0f, -8.0f, 8.0f);
    }
    return std::clamp(-pid_output * 35.0f, -30.0f, 30.0f);
}

#endif  // LANE_DETECT_HPP
