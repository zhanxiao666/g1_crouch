import numpy as np
import time
import argparse
import os 
import sys
import threading
import pinocchio as pin  

# 1. 导入系统级 LCM 库
import lcm

# 路径配置
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

# 2. 导入 LCM 消息类型 
from lcm_types.arm_action_lcmt import arm_action_lcmt
from lcm_types.body_control_data_lcmt import body_control_data_lcmt

# 3. 仅保留纯算法的 IK 求解器
from teleop.robot_control.robot_arm_ik import G1_29_ArmIK

# ==============================================================================
# LCM 状态接收器
# ==============================================================================
class LCMRobotState:
    def __init__(self):
        self.lc = lcm.LCM()
        self.current_body_q = None
        self.current_arm_q = None
        self.current_arm_dq = None
        self.current_base_quat_w = np.array([1.0, 0.0, 0.0, 0.0])
        
        self.lc.subscribe("body_control_data", self.state_handler)
        self.receive_thread = threading.Thread(target=self.lcm_spin)
        self.receive_thread.daemon = True
        self.receive_thread.start()

    def state_handler(self, channel, data):
        msg = body_control_data_lcmt.decode(data)
        self.current_body_q = np.array(msg.q, dtype=float)
        self.current_arm_q = np.array(msg.q[15:29])
        self.current_arm_dq = np.array(msg.qd[15:29])
        self.current_base_quat_w = np.array(msg.root_quat_w, dtype=float)

    def lcm_spin(self):
        while True:
            self.lc.handle()

# ==============================================================================
# 安全空间轨迹规划器 (带精确的状态机)
# ==============================================================================
class SafeTrajectoryPlanner:
    def __init__(self, real_L_pose: pin.SE3, real_R_pose: pin.SE3):
        self.start_time = time.time()
        
        # 时间参数
        self.transition_duration = 3.0  # 起步过渡期
        self.cycle_time = 4.0           # 执行单次作业的时间
        # 核心：准确计算作业完成的绝对时间点
        self.total_duration = self.transition_duration + self.cycle_time 

        # 真实起点
        self.real_L_pose = real_L_pose.copy()
        self.real_R_pose = real_R_pose.copy()

        # 作业基准点
        self.task_L_base = pin.SE3(pin.Quaternion(1, 0, 0, 0), np.array([0.25,  0.35, 0.1]))
        self.task_R_base = pin.SE3(pin.Quaternion(1, 0, 0, 0), np.array([0.25, -0.05, 0.1]))
        
        # 作业终点
        self.task_R_end_pos = np.array([0.25, -0.25, 0.2])

    def get_planned_pose(self) -> tuple[np.array, np.array, bool]:
        t = time.time() - self.start_time
        
        is_done = False # 结束标志位

        # 阶段 1：安全过渡期 (0 ~ 3.0秒)
        if t < self.transition_duration:
            alpha = t / self.transition_duration
            smooth_alpha = 3 * (alpha ** 2) - 2 * (alpha ** 3)
            L_target = pin.SE3.Interpolate(self.real_L_pose, self.task_L_base, smooth_alpha)
            R_target = pin.SE3.Interpolate(self.real_R_pose, self.task_R_base, smooth_alpha)
            
        # 阶段 2：正式作业期 (3.0 ~ 7.0秒)
        elif t <= self.total_duration:
            t_task = t - self.transition_duration
            L_target = self.task_L_base.copy()
            alpha_task = (np.sin(2 * np.pi * t_task / self.cycle_time - np.pi/2) + 1) / 2
            R_target_pos = (1 - alpha_task) * self.task_R_base.translation + alpha_task * self.task_R_end_pos
            R_target = self.task_R_base.copy()
            R_target.translation = R_target_pos
            
        # 阶段 3：完美收工 (> 7.0秒)
        else:
            L_target = self.task_L_base.copy()
            R_target = self.task_R_base.copy()
            is_done = True # 告诉主循环：我做完了，且现在稳稳停在基准点！

        return L_target.homogeneous, R_target.homogeneous, is_done

def smoothstep(alpha: float) -> float:
    alpha = float(np.clip(alpha, 0.0, 1.0))
    return 3.0 * alpha ** 2 - 2.0 * alpha ** 3

def publish_right_arm(lc_pub, right_arm_q):
    msg = arm_action_lcmt()
    msg.act = [0.0] * 14
    msg.act[0:7] = np.asarray(right_arm_q, dtype=float).tolist()
    lc_pub.publish("arm_action_data", msg.encode())

def quat_wxyz_to_rot(quat_wxyz):
    quat_wxyz = np.asarray(quat_wxyz, dtype=float)
    norm = np.linalg.norm(quat_wxyz)
    if norm < 1e-6:
        return np.eye(3)
    quat_wxyz = quat_wxyz / norm
    return pin.Quaternion(
        quat_wxyz[0],
        quat_wxyz[1],
        quat_wxyz[2],
        quat_wxyz[3],
    ).toRotationMatrix()

def rot_y(theta_rad):
    c = np.cos(theta_rad)
    s = np.sin(theta_rad)
    return np.array([
        [c, 0.0, s],
        [0.0, 1.0, 0.0],
        [-s, 0.0, c],
    ], dtype=float)

def get_torso_pitch_rad(current_body_q):
    # For G1_29, q[14] is waist_pitch_joint. Since the IK target is expressed in
    # the waist frame, this joint angle is the torso forward-lean we need.
    return float(current_body_q[14])

def apply_torso_pitch_compensation(target_homogeneous, torso_pitch_rad):
    # The IK target is expressed in the waist frame. When the torso pitches
    # around the waist Y axis, rotate the upright-reference target by the same
    # pitch so the target remains consistent with the straight-torso plan.
    comp_rot = rot_y(-torso_pitch_rad)

    target = np.array(target_homogeneous, dtype=float, copy=True)
    target[:3, :3] = comp_rot @ target[:3, :3]
    target[:3, 3] = comp_rot @ target[:3, 3]
    return target

# ==============================================================================
# 主控制逻辑
# ==============================================================================
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--frequency', type=float, default=30.0)
    parser.add_argument('--return-duration', type=float, default=3.0,
                        help='Seconds used to ease the right arm back to zero after the IK task.')
    parser.add_argument('--disable-torso-compensation', action='store_true',
                        help='Disable IMU torso tilt compensation for IK targets.')
    args = parser.parse_args()

    # 1. 启动 LCM 状态接收器
    state_reader = LCMRobotState()
    while state_reader.current_arm_q is None:
        time.sleep(0.05)

    # 2. 初始化 IK 求解器
    arm_ik = G1_29_ArmIK()
    lc_pub = lcm.LCM() 

    # 3. 算出当前真实位姿
    q_init = state_reader.current_arm_q
    pin.framesForwardKinematics(arm_ik.reduced_robot.model, arm_ik.reduced_robot.data, q_init)
    pin.updateFramePlacements(arm_ik.reduced_robot.model, arm_ik.reduced_robot.data)
    real_L_pose = arm_ik.reduced_robot.data.oMf[arm_ik.L_hand_id]
    real_R_pose = arm_ik.reduced_robot.data.oMf[arm_ik.R_hand_id]

    planner = SafeTrajectoryPlanner(real_L_pose, real_R_pose)
    
    print("\n[INFO] 🤖 自动作业模式：开始执行平滑启动及轨迹运动...")
    if args.disable_torso_compensation:
        print("[INFO] IMU 躯干倾斜补偿：关闭")
    else:
        print("[INFO] 躯干前倾补偿：开启，使用 waist_pitch_joint 并按已校准方向补偿")

    running = True
    last_right_arm_cmd = np.zeros(7)
    last_pitch_print_time = 0.0
    
    try:
        while running:
            loop_start = time.time()

            # 从规划器获取位姿，以及“是否完成”的标志位
            left_target, right_target, is_done = planner.get_planned_pose()
            if not args.disable_torso_compensation:
                torso_pitch_rad = get_torso_pitch_rad(state_reader.current_body_q)
                left_target = apply_torso_pitch_compensation(left_target, torso_pitch_rad)
                right_target = apply_torso_pitch_compensation(right_target, torso_pitch_rad)

                now = time.time()
                if now - last_pitch_print_time > 1.0:
                    print(f"[INFO] torso pitch from waist joint: {np.degrees(torso_pitch_rad):.2f} deg")
                    last_pitch_print_time = now

            # 正常求解发送
            current_q = state_reader.current_arm_q
            current_dq = state_reader.current_arm_dq
            sol_q, _ = arm_ik.solve_ik(left_target, right_target, current_q, current_dq)

            if sol_q is not None:
                last_right_arm_cmd = np.array(sol_q[7:14], dtype=float)
                publish_right_arm(lc_pub, last_right_arm_cmd)

            # 【核心修改】精准检测退出时机
            if is_done:
                print(f"[INFO] ✅ 1次作业周期精准完成 ({planner.total_duration}秒)，准备退出...")
                print(f"[INFO] 右臂开始平滑回零，耗时 {args.return_duration:.1f} 秒...")
                return_start = time.time()
                return_period = 1.0 / args.frequency
                while True:
                    elapsed = time.time() - return_start
                    alpha = smoothstep(elapsed / max(args.return_duration, 1e-6))
                    right_arm_cmd = (1.0 - alpha) * last_right_arm_cmd
                    publish_right_arm(lc_pub, right_arm_cmd)
                    if alpha >= 1.0:
                        break
                    time.sleep(return_period)
                publish_right_arm(lc_pub, np.zeros(7))
                time.sleep(0.2)
                break # 安全退出循环

            loop_cost = time.time() - loop_start
            sleep_time = max(0, (1.0 / args.frequency) - loop_cost)
            time.sleep(sleep_time)

    except KeyboardInterrupt:
        pass
    finally:
        print("[INFO] 手臂作业完美结束，控制权交还给底盘。")

if __name__ == '__main__':
    main()
