#include "FSM/State_RLBase.h"
#include "unitree_articulation.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"

#include <chrono>

namespace isaaclab
{
namespace
{
float g_target_height = 0.76f;
std::vector<float> g_left_arm_cmd(7, 0.0f);
bool g_left_arm_initialized = false;
std::vector<float> g_right_arm_cmd(7, 0.0f);
bool g_right_arm_initialized = true;
int g_arm_debug_counter = 0;
std::chrono::steady_clock::time_point g_last_run_time = std::chrono::steady_clock::now();

float apply_deadzone(float value, float deadzone)
{
    return std::abs(value) < deadzone ? 0.0f : value;
}

float move_towards(float current, float target, float max_delta)
{
    float delta = target - current;
    if (std::abs(delta) <= max_delta) {
        return target;
    }
    return current + std::copysign(max_delta, delta);
}
}

REGISTER_OBSERVATION(velocity_height_commands)
{
    static auto cfg = env->cfg["commands"]["twist"]["ranges"];
    static float target_height = cfg["base_height"][1].as<float>();
    static bool prev_up = false;
    static bool prev_down = false;
    float stick_deadzone = env->cfg["commands"]["twist"]["stick_deadzone"].as<float>(0.10f);

    auto& joystick = env->robot->data.joystick;
    bool up_pressed = joystick->up();
    bool down_pressed = joystick->down();

    float height_step = env->cfg["commands"]["twist"]["height_step"].as<float>(0.02f);
    if (up_pressed && !prev_up) {
        target_height += height_step;
    }
    if (down_pressed && !prev_down) {
        target_height -= height_step;
    }
    prev_up = up_pressed;
    prev_down = down_pressed;

    target_height = std::clamp(
        target_height,
        cfg["base_height"][0].as<float>(),
        cfg["base_height"][1].as<float>()
    );
    env->robot->data.target_height_cmd = target_height;
    g_target_height = target_height;

    std::vector<float> obs(4);
    obs[0] = std::clamp(apply_deadzone(joystick->ly(), stick_deadzone), cfg["lin_vel_x"][0].as<float>(), cfg["lin_vel_x"][1].as<float>());
    obs[1] = std::clamp(apply_deadzone(-joystick->lx(), stick_deadzone), cfg["lin_vel_y"][0].as<float>(), cfg["lin_vel_y"][1].as<float>());
    obs[2] = std::clamp(apply_deadzone(-joystick->rx(), stick_deadzone), cfg["ang_vel_z"][0].as<float>(), cfg["ang_vel_z"][1].as<float>());
    obs[3] = target_height;
    return obs;
}

}

State_RLBase::State_RLBase(int state_mode, std::string state_string)
: FSMState(state_mode, state_string)
{
    auto cfg = param::config["FSM"][state_string];
    auto policy_dir = param::parser_policy_dir(cfg["policy_dir"].as<std::string>());

    env = std::make_unique<isaaclab::ManagerBasedRLEnv>(
        YAML::LoadFile(policy_dir / "params" / "deploy.yaml"),
        std::make_shared<unitree::BaseArticulation<LowState_t::SharedPtr>>(FSMState::lowstate)
    );
    env->alg = std::make_unique<isaaclab::OrtRunner>(policy_dir / "exported" / "policy.onnx");
    right_arm_ik_lcm = std::make_unique<RightArmIKLCMBridge>(env->cfg["commands"]["twist"]);

    this->registered_checks.emplace_back(
        std::make_pair(
            [&]()->bool{ return isaaclab::mdp::bad_orientation(env.get(), 1.0); },
            FSMStringMap.right.at("Passive")
        )
    );
}

void State_RLBase::run()
{
    auto now = std::chrono::steady_clock::now();
    float dt = std::chrono::duration<float>(now - isaaclab::g_last_run_time).count();
    isaaclab::g_last_run_time = now;
    dt = std::clamp(dt, 0.0f, 0.05f);

    auto action = env->action_manager->processed_actions();
    if (right_arm_ik_lcm && right_arm_ik_lcm->enabled()) {
        right_arm_ik_lcm->publish_state(
            env->robot->data.joint_pos,
            env->robot->data.joint_vel,
            env->robot->data.root_quat_w
        );
    }

    bool debug_arm_actions = env->cfg["commands"]["twist"]["debug_arm_actions"].as<bool>(false);
    if (debug_arm_actions && isaaclab::g_target_height < 0.74f && action.size() >= 29)
    {
        if ((isaaclab::g_arm_debug_counter++ % 25) == 0)
        {
            spdlog::info(
                "Squat arm actions | L:[{:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}] "
                "R:[{:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}] height_cmd={:.3f}",
                action[15], action[16], action[17], action[18], action[19], action[20], action[21],
                action[22], action[23], action[24], action[25], action[26], action[27], action[28],
                isaaclab::g_target_height
            );
        }
    }

    float squat_threshold = env->cfg["commands"]["twist"]["right_arm_external_height_threshold"].as<float>(0.74f);
    float ik_blend = env->cfg["commands"]["twist"]["right_arm_ik_blend"].as<float>(
        env->cfg["commands"]["twist"]["right_arm_external_blend"].as<float>(0.20f));
    float zero_max_speed = env->cfg["commands"]["twist"]["right_arm_zero_max_speed"].as<float>(0.35f);
    ik_blend = std::clamp(ik_blend, 0.0f, 1.0f);
    zero_max_speed = std::max(0.0f, zero_max_speed);

    if (!isaaclab::g_right_arm_initialized)
    {
        for (int i = 0; i < 7; ++i)
        {
            int joint_id = 22 + i;
            if (joint_id < action.size()) {
                isaaclab::g_right_arm_cmd[i] = action[joint_id];
            }
        }
        isaaclab::g_right_arm_initialized = true;
    }

    bool right_arm_external = isaaclab::g_target_height < squat_threshold;
    if (right_arm_external)
    {
        std::array<float, 7> right_arm_target{};
        bool has_ik_target = right_arm_ik_lcm &&
            right_arm_ik_lcm->latest_right_arm_target(right_arm_target);

        for (int i = 0; i < 7; ++i)
        {
            int joint_id = 22 + i;
            if (joint_id >= action.size()) {
                break;
            }
            float target = has_ik_target ? right_arm_target[i] : 0.0f;
            if (has_ik_target) {
                isaaclab::g_right_arm_cmd[i] =
                    (1.0f - ik_blend) * isaaclab::g_right_arm_cmd[i] + ik_blend * target;
            } else {
                isaaclab::g_right_arm_cmd[i] = isaaclab::move_towards(
                    isaaclab::g_right_arm_cmd[i],
                    target,
                    zero_max_speed * dt
                );
            }
            action[joint_id] = isaaclab::g_right_arm_cmd[i];
        }
    }
    else
    {
        for (int i = 0; i < 7; ++i)
        {
            int joint_id = 22 + i;
            if (joint_id >= action.size()) {
                break;
            }
            isaaclab::g_right_arm_cmd[i] = action[joint_id];
        }
    }

    for (int i = 0; i < env->robot->data.joint_ids_map.size(); i++) {
        lowcmd->msg_.motor_cmd()[env->robot->data.joint_ids_map[i]].q() = action[i];
    }
}
