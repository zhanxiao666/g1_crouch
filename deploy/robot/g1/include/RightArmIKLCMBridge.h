#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <yaml-cpp/yaml.h>

#ifdef UNITREE_RL_ENABLE_ARM_IK_LCM
#include <lcm/lcm-cpp.hpp>
#include "lcm_types/arm_action_lcmt.hpp"
#include "lcm_types/body_control_data_lcmt.hpp"
#endif

class RightArmIKLCMBridge
{
public:
    explicit RightArmIKLCMBridge(const YAML::Node& cfg);
    ~RightArmIKLCMBridge();

    RightArmIKLCMBridge(const RightArmIKLCMBridge&) = delete;
    RightArmIKLCMBridge& operator=(const RightArmIKLCMBridge&) = delete;

    bool enabled() const { return enabled_; }
    void publish_state(const Eigen::VectorXf& q, const Eigen::VectorXf& dq,
                       const Eigen::Quaternionf& root_quat_w);
    bool latest_right_arm_target(std::array<float, 7>& target);

private:
#ifdef UNITREE_RL_ENABLE_ARM_IK_LCM
    void handle_arm_action(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
                           const arm_action_lcmt* msg);
    void spin();
#endif

    bool enabled_ = false;
    int right_arm_source_offset_ = 0;
    double timeout_s_ = 0.2;
    std::string action_channel_ = "arm_action_data";
    std::string state_channel_ = "body_control_data";

    std::mutex mutex_;
    std::array<float, 7> right_arm_target_{};
    std::chrono::steady_clock::time_point last_target_time_{};
    bool has_target_ = false;

#ifdef UNITREE_RL_ENABLE_ARM_IK_LCM
    std::unique_ptr<lcm::LCM> lcm_;
    std::thread thread_;
    std::atomic<bool> running_{false};
#endif
};
