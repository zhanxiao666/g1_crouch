//状态机状态基类
#pragma once

#include "Types.h"
#include "param.h"
#include "FSM/BaseState.h"
#include "isaaclab/devices/keyboard/keyboard.h"
#include "unitree_joystick_dsl.hpp"

class FSMState : public BaseState
{
public:
    FSMState(int state, std::string state_string) 
    : BaseState(state, state_string) 
    {
        spdlog::info("Initializing State_{} ...", state_string);

        auto transitions = param::config["FSM"][state_string]["transitions"];

        if(transitions)
        {
            auto transition_map = transitions.as<std::map<std::string, std::string>>();

            for(auto it = transition_map.begin(); it != transition_map.end(); ++it)
            {
                std::string target_fsm = it->first;
                if(!FSMStringMap.right.count(target_fsm))
                {
                    spdlog::warn("FSM State_'{}' not found in FSMStringMap!", target_fsm);
                    continue;
                }

                int fsm_id = FSMStringMap.right.at(target_fsm);

                std::string condition = it->second;
                unitree::common::dsl::Parser p(condition);
                auto ast = p.Parse();
                auto func = unitree::common::dsl::Compile(*ast);
                registered_checks.emplace_back(
                    std::make_pair(
                        [func]()->bool{ return func(FSMState::lowstate->joystick); },
                        fsm_id
                    )
                );
            }
        }

        // G1 keyboard shortcuts for simulator bring-up without a physical joystick.
        if (keyboard)
        {
            auto add_keyboard_transition = [&](const std::string& key_name, const std::string& target_fsm) {
                if (!FSMStringMap.right.count(target_fsm))
                {
                    return;
                }
                int fsm_id = FSMStringMap.right.at(target_fsm);
                registered_checks.emplace_back(
                    std::make_pair(
                        [key_name]()->bool {
                            return FSMState::keyboard && FSMState::keyboard->on_pressed &&
                                   FSMState::keyboard->key() == key_name;
                        },
                        fsm_id
                    )
                );
            };

            if (state_string == "Passive")
            {
                add_keyboard_transition("up", "FixStand");
            }
            else if (state_string == "FixStand")
            {
                add_keyboard_transition("b", "Passive");
                add_keyboard_transition("r", "Velocity");
            }
            else if (state_string == "Velocity")
            {
                add_keyboard_transition("b", "Passive");
            }
        }

        // register for all states
        registered_checks.emplace_back(
            std::make_pair(
                []()->bool{ return lowstate->isTimeout(); },
                FSMStringMap.right.at("Passive")
            )
        );
    }

    void pre_run()
    {
        lowstate->update();
        if(keyboard) keyboard->update();
    }

    void post_run()
    {
        lowcmd->unlockAndPublish();
    }

    static std::unique_ptr<LowCmd_t> lowcmd;
    static std::shared_ptr<LowState_t> lowstate;
    static std::shared_ptr<Keyboard> keyboard;
};
