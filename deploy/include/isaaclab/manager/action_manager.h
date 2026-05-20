// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include "isaaclab/envs/manager_based_rl_env.h"
#include "isaaclab/manager/manager_term_cfg.h"
#include <numeric>
#include <deque>
#include <random>

namespace isaaclab
{

class ActionTerm 
{
public:
    ActionTerm(YAML::Node cfg, ManagerBasedRLEnv* env): cfg(cfg), env(env) {}

    virtual int action_dim() = 0;
    virtual std::vector<float> raw_actions() = 0;
    virtual std::vector<float> processed_actions() = 0;
    virtual void process_actions(std::vector<float> actions) = 0;
    virtual void reset(){};

protected:
    YAML::Node cfg;
    ManagerBasedRLEnv* env;
};

inline std::map<std::string, std::function<std::unique_ptr<ActionTerm>(YAML::Node, ManagerBasedRLEnv*)>>& actions_map() {
    static std::map<std::string, std::function<std::unique_ptr<ActionTerm>(YAML::Node, ManagerBasedRLEnv*)>> instance;
    return instance;
}

#define REGISTER_ACTION(name) \
    inline struct name##_registrar { \
        name##_registrar() { \
            actions_map()[#name] = [](YAML::Node cfg, ManagerBasedRLEnv* env) { \
                return std::make_unique<name>(cfg, env); \
            }; \
        } \
    } name##_registrar_instance;

class ActionManager
{
public:
    ActionManager(YAML::Node cfg, ManagerBasedRLEnv* env, YAML::Node delay_cfg = YAML::Node())
    : cfg(cfg), env(env), delay_cfg(delay_cfg)
    {
        _prepare_terms();
        _action.resize(total_action_dim(), 0.0f);
        _executed_action = _action;
        _parse_delay_cfg();
    }

    void reset()
    {
        _action.assign(total_action_dim(), 0.0f);
        _executed_action = _action;
        _action_history.clear();
        for(auto & term : _terms)
        {
            term->reset();
        }
    }

    std::vector<float> action()
    {
        return _executed_action;
    }

    std::vector<float> processed_actions()
    {
        std::vector<float> actions;
        for(auto & term : _terms)
        {
            auto term_action = term->processed_actions();
            actions.insert(actions.end(), term_action.begin(), term_action.end());
        }
        return actions;
    }

    void process_action(std::vector<float> action)
    {
        _action = action;
        _executed_action = _apply_action_delay(action);
        int idx = 0;
        for(auto & term : _terms)
        {
            auto term_action = std::vector<float>(_executed_action.begin() + idx, _executed_action.begin() + idx + term->action_dim());
            term->process_actions(term_action);
            idx += term->action_dim();
        }
    }

    int total_action_dim()
    {
        auto dims = action_dim();
        
        return std::accumulate(dims.begin(), dims.end(), 0);
    }

    std::vector<int> action_dim()
    {
        std::vector<int> dims;
        for (auto & term : _terms)
        {
            dims.push_back(term->action_dim());
        }
        return dims;
    }

    YAML::Node cfg;
    ManagerBasedRLEnv* env;
    YAML::Node delay_cfg;

private:
    void _parse_delay_cfg()
    {
        try {
            _delay_enabled = delay_cfg["enabled"].as<bool>(false);
            _delay_max_lag = delay_cfg["max_lag"].as<int>(0);
            _delay_hold_prob = delay_cfg["hold_prob"].as<float>(0.0f);
            _delay_max_lag = std::max(0, _delay_max_lag);
            _delay_hold_prob = std::clamp(_delay_hold_prob, 0.0f, 1.0f);
        } catch(const std::exception&) {
            _delay_enabled = false;
        }
    }

    std::vector<float> _apply_action_delay(const std::vector<float>& action)
    {
        if(!_delay_enabled) {
            return action;
        }

        static thread_local std::mt19937 rng(std::random_device{}());
        std::uniform_real_distribution<float> unit_dist(0.0f, 1.0f);

        _action_history.push_back(action);
        while(_action_history.size() > static_cast<size_t>(_delay_max_lag + 1)) {
            _action_history.pop_front();
        }

        if(!_executed_action.empty() && unit_dist(rng) < _delay_hold_prob) {
            return _executed_action;
        }

        int lag = 0;
        if(_delay_max_lag > 0) {
            std::uniform_int_distribution<int> lag_dist(0, _delay_max_lag);
            lag = lag_dist(rng);
        }
        lag = std::min<int>(lag, static_cast<int>(_action_history.size()) - 1);
        return _action_history[_action_history.size() - 1 - lag];
    }

    void _prepare_terms()
    {
        for(auto it = this->cfg.begin(); it != this->cfg.end(); ++it)
        {
            std::string action_name = it->first.as<std::string>();
            if(actions_map().find(action_name) == actions_map().end())
            {
                throw std::runtime_error("Action term '" + action_name + "' is not registered.");
            }

            auto term = actions_map()[action_name](it->second, env);
            _terms.push_back(std::move(term));
        }
    }

    std::vector<float> _action;
    std::vector<float> _executed_action;
    std::deque<std::vector<float>> _action_history;
    bool _delay_enabled = false;
    int _delay_max_lag = 0;
    float _delay_hold_prob = 0.0f;
    std::vector<std::unique_ptr<ActionTerm>> _terms;
};

};
