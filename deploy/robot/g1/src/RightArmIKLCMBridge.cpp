#include "RightArmIKLCMBridge.h"

#include <algorithm>

#ifdef UNITREE_RL_ENABLE_ARM_IK_LCM
#include <cstring>
#endif

RightArmIKLCMBridge::RightArmIKLCMBridge(const YAML::Node& cfg)
{
    right_arm_source_offset_ = cfg["right_arm_ik_source_offset"].as<int>(0);
    timeout_s_ = cfg["right_arm_ik_timeout"].as<double>(0.2);
    action_channel_ = cfg["right_arm_ik_action_channel"].as<std::string>("arm_action_data");
    state_channel_ = cfg["right_arm_ik_state_channel"].as<std::string>("body_control_data");

#ifdef UNITREE_RL_ENABLE_ARM_IK_LCM
    enabled_ = true;
    right_arm_source_offset_ = std::clamp(right_arm_source_offset_, 0, 7);
    lcm_ = std::make_unique<lcm::LCM>();
    if (!lcm_->good()) {
        enabled_ = false;
        return;
    }

    lcm_->subscribe(action_channel_, &RightArmIKLCMBridge::handle_arm_action, this);
    running_ = true;
    thread_ = std::thread(&RightArmIKLCMBridge::spin, this);
#else
    enabled_ = false;
#endif
}

RightArmIKLCMBridge::~RightArmIKLCMBridge()
{
#ifdef UNITREE_RL_ENABLE_ARM_IK_LCM
    running_ = false;
    if (thread_.joinable()) {
        thread_.join();
    }
#endif
}

void RightArmIKLCMBridge::publish_state(const Eigen::VectorXf& q, const Eigen::VectorXf& dq,
                                        const Eigen::Quaternionf& root_quat_w)
{
#ifdef UNITREE_RL_ENABLE_ARM_IK_LCM
    if (!enabled_ || !lcm_) {
        return;
    }

    body_control_data_lcmt msg{};
    const int n = std::min<int>(29, std::min(q.size(), dq.size()));
    for (int i = 0; i < n; ++i) {
        msg.q[i] = q[i];
        msg.qd[i] = dq[i];
    }
    msg.root_quat_w[0] = root_quat_w.w();
    msg.root_quat_w[1] = root_quat_w.x();
    msg.root_quat_w[2] = root_quat_w.y();
    msg.root_quat_w[3] = root_quat_w.z();
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    msg.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(now).count();
    lcm_->publish(state_channel_, &msg);
#else
    (void)q;
    (void)dq;
    (void)root_quat_w;
#endif
}

bool RightArmIKLCMBridge::latest_right_arm_target(std::array<float, 7>& target)
{
    if (!enabled_) {
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_target_) {
        return false;
    }

    const auto age = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - last_target_time_).count();
    if (age > timeout_s_) {
        return false;
    }

    target = right_arm_target_;
    return true;
}

#ifdef UNITREE_RL_ENABLE_ARM_IK_LCM
void RightArmIKLCMBridge::handle_arm_action(const lcm::ReceiveBuffer*,
                                            const std::string&,
                                            const arm_action_lcmt* msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    for (int i = 0; i < 7; ++i) {
        right_arm_target_[i] = static_cast<float>(msg->act[right_arm_source_offset_ + i]);
    }
    last_target_time_ = std::chrono::steady_clock::now();
    has_target_ = true;
}

void RightArmIKLCMBridge::spin()
{
    while (running_) {
        lcm_->handleTimeout(10);
    }
}
#endif
