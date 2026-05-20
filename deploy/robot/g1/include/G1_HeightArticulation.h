// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include "Types.h"
#include "isaaclab/assets/articulation/articulation.h"

class G1HeightArticulation : public isaaclab::Articulation
{
public:
    G1HeightArticulation(
        LowState_t::SharedPtr lowstate_,
        SportState_t::SharedPtr sportstate_)
    : lowstate(std::move(lowstate_)), sportstate(std::move(sportstate_))
    {
        data.joystick = &lowstate->joystick;
    }

    void update() override
    {
        std::lock_guard<std::mutex> lock(lowstate->mutex_);

        for (int i = 0; i < 3; ++i) {
            data.root_ang_vel_b[i] = lowstate->msg_.imu_state().gyroscope()[i];
        }

        data.root_quat_w = Eigen::Quaternionf(
            lowstate->msg_.imu_state().quaternion()[0],
            lowstate->msg_.imu_state().quaternion()[1],
            lowstate->msg_.imu_state().quaternion()[2],
            lowstate->msg_.imu_state().quaternion()[3]
        );
        data.projected_gravity_b = data.root_quat_w.conjugate() * data.GRAVITY_VEC_W;

        for (int i = 0; i < data.joint_ids_map.size(); ++i) {
            data.joint_pos[i] = lowstate->msg_.motor_state()[data.joint_ids_map[i]].q();
            data.joint_vel[i] = lowstate->msg_.motor_state()[data.joint_ids_map[i]].dq();
        }

        data.root_lin_vel_b.setZero();
        if (sportstate && !sportstate->isTimeout()) {
            std::lock_guard<std::mutex> sport_lock(sportstate->mutex_);
            Eigen::Vector3f root_lin_vel_w = sportstate->velocity();
            data.root_lin_vel_b = data.root_quat_w.conjugate() * root_lin_vel_w;
        }
    }

private:
    LowState_t::SharedPtr lowstate;
    SportState_t::SharedPtr sportstate;
};
