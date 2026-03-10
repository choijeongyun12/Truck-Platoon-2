#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <algorithm>

class PIDController {
public:
    PIDController(float kp = 0.5f, float ki = 0.0095f, float kd = 0.03f, float i_limit = 1.0f)
        : kp_(kp),
          ki_(ki),
          kd_(kd),
          i_limit_(i_limit),
          prev_error_(0.0f),
          integral_(0.0f),
          initialized_(false) {}

    void reset() {
        prev_error_ = 0.0f;
        integral_ = 0.0f;
        initialized_ = false;
    }

    float compute(float error, float dt = 1.0f) {
        dt = std::max(1e-3f, dt);
        if (!initialized_) {
            prev_error_ = error;
            initialized_ = true;
        }

        integral_ += error * dt;
        if (i_limit_ > 0.0f) {
            integral_ = std::clamp(integral_, -i_limit_, i_limit_);
        }

        const float derivative = (error - prev_error_) / dt;
        const float output = (kp_ * error) + (ki_ * integral_) + (kd_ * derivative);
        prev_error_ = error;
        return output;
    }

private:
    float kp_;
    float ki_;
    float kd_;
    float i_limit_;
    float prev_error_;
    float integral_;
    bool initialized_;
};

#endif  // PID_CONTROLLER_HPP
