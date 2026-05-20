# 宇树机器人的训练库
### 介绍
```
训练库基于mjlab。不做框架处理，仅给出训练代码
```
这里是mjlab的部署代码的整体结构。
在这个项目中，强化学习网络的部署是基于有限状态机 (FSM) 架构完成的。为了保证机器人底层的控制频率（1000Hz）和神经网络的推理频率（例如50Hz）互不干扰，代码采用了优秀的双线程架构。

## arm文件夹
```这是负责逆解的代码。其中的ik_trajectory_demo.py就是例程。
lcm是用来在不同语言间进行通信的。发送关节角度给机器人，接收机器人的关节角度。
robot_arm.py是机器人的底层控制代码。可以直接运行。但是本项目不用。我们只用robot_arm_ik.py的逆解来进行计算。得到的关节角实时通过lcm发送给机器人。
```
## 遥控器按键
```
L2 + 上：Passive -> FixStand
R2 + A：FixStand -> Velocity
L2 + B：退回 Passive 也可做急停按钮
下方向键：逐步下蹲
上方向键：逐步升高
```
## g1_crouch/deploy/include/isaaclab 解释

### 这个文件夹实现了从训练环境倒实际部署的桥梁
```sh
📁 [algorithms/algorithms.h]
#加载onnx网络并进行推理，使用 ONNX Runtime 加载并执行训练好的策略网络

class OrtRunner : public Algorithms {
    # 加载 ONNX 模型
    session = std::make_unique<Ort::Session>(env, model_path.c_str(), session_options);
    
    # 执行推理
    std::vector<float> act(std::unordered_map<std::string, std::vector<float>> obs) {
        # 将观测数据转换为 ONNX 输入
        # 运行模型推理
        # 返回动作输出
```
```sh
📁 [envs/manager_based_rl_env.h]
#复现训练时的环境接口，确保观测和动作的处理逻辑与训练时完全一致

class ManagerBasedRLEnv {
    # 从 deploy.yaml 加载配置
    ManagerBasedRLEnv(YAML::Node cfg, std::shared_ptr<Articulation> robot_)
    
    # 每一步的执行流程
    void step() {
        auto obs = observation_manager->compute();  # 计算观测
        auto action = alg->act(obs);                # 策略推理
        action_manager->process_action(action);     # 处理动作
```

```sh
📁 [manager/observation_manager.h]
#根据  deploy.yaml 配置构建观测空间
```

```sh
📁 [manager/action_manager.h]
📁 [envs/mdp/actions/joint_actions.h]
#动作管理器：
✅ 将策略输出的归一化动作转换为实际关节角度
✅ 应用训练时使用的 scale 和 offset
✅ 确保动作在安全范围内
```
```sh
#观测函数实现
📁 [envs/mdp/observations/observations.h]
#实现具体的观测数据采集逻辑，从机器人底层状态或外部设备（手柄）获取数据
```
## deploy/include/FSM 解释
### 这个文件夹实现了有限状态机（Finite State Machine, FSM）架构，它负责管理机器人的不同控制模式及其切换逻辑。

📄 [BaseState.h]
```sh
✅ 定义所有状态的统一接口
✅ 提供状态生命周期管理
✅ 支持动态注册安全检查条件
✅ 实现工厂模式，便于扩展新状态
```

📄 [FSMState.h]
```sh
✅ 解析 YAML 配置中的状态转换规则
✅ 将手柄按键映射到状态切换条件
✅ 添加键盘快捷键支持（仿真调试用）
✅ 实现全局安全保护（通信超时自动降级）

```

📄 [State_Passive.h]
```sh
🔒 安全模式：机器人处于零力矩状态
🛡️ 紧急状态：检测到危险时自动切换到此状态
🔧 初始化状态：系统启动时的默认状态
```

📄 [State_FixStand.h]
```sh
🧍 过渡状态：从被动模式平滑过渡到站立姿态
🎯 准备状态：为 RL 控制做准备（机器人先站稳）
⏱️ 时间插值：避免突然的姿态跳变导致冲击
关节角度配置在depoly、robots/g1/config/config.yaml中
```

📄 [State_RLBase.h]
```sh
🤖 核心控制状态：运行训练好的 RL 策略
⚡ 多线程架构：独立线程保证实时性（通常 50Hz）
🔄 完整环境复现：使用 isaaclab 框架处理观测和动作
🛑 安全退出：确保线程正确清理资源
```

📄 [CtrlFSM.h]
```sh
🎮 中央调度器：管理所有状态的创建、切换和执行
⏰ 高频控制循环：1ms 周期（1kHz）确保实时响应
🔀 动态状态切换：根据注册的条件自动切换状态
📋 配置驱动：通过 YAML 文件定义状态和转换规则
```
FSM的设计原则就是安全优先。
```
┌──────────────────────────────────────────────┐
│              系统架构图                        │
└──────────────────────────────────────────────┘

┌─────────────┐
│  main.cpp   │ 程序入口
└──────┬──────┘
       │ 初始化
       ▼
┌─────────────┐         ┌──────────────────┐
│  CtrlFSM    │◄────────│  config.yaml     │
│  (调度器)    │  加载配置│  (状态定义）       │
└──────┬──────┘         └──────────────────┘
       │ 管理
       ├─────────────────────────────────────┐
       ▼                   ▼                 ▼
┌─────────────┐  ┌──────────────┐  ┌──────────────┐
│State_Passive│  │State_FixStand│  │State_RLBase  │
│             │  │              │  │              │
│             │  │              │  ├──────────────┤
│             │  │              │  │ isaaclab/    │
│             │  │              │  │  ├ algorithms│
│             │  │              │  │  ├ envs      │
│             │  │              │  │  └ manager   │
└─────────────┘  └──────────────┘  └──────────────┘
       │                   │                 │
       └───────────────────┴─────────────────┘
                         │
                         ▼
              ┌──────────────────┐
              │  DDS 通信层       │
              │  (unitree_sdk2)  │
              └──────────────────┘
```
## include其他文件
📄 1. unitree_joystick_dsl.hpp -

这是一个手柄按键表达式解析器，允许用人类可读的字符串描述复杂的按键组合逻辑。
```sh

#----------------
// 按钮键
"A", "B", "X", "Y", "LB", "RB", "LT", "RT"
"start", "back", "LS", "RS"

// 方向键
"up", "down", "left", "right"

// 摇杆轴
"LX", "LY", "RX", "RY"

// 功能键
"F1", "F2"
#----------------
"+"  → AND (同时满足)
"|"  → OR  (任一满足)
"!"  → NOT (取反)
"()" → 分组优先级
"."  → 状态修饰符
#-----------------
"A.on_pressed"    // A刚按下（单帧触发）
"A.on_released"   // A刚释放（单帧触发）
"A.pressed"       // A持续按下
"LT(2s)"          // LT长按超过2秒
```
📄 2. LinearInterpolator.h 
实现多维数据的线性插值，用于 FixStand 状态的平滑姿态过渡。

📄 3. param.h 
统一管理程序配置、日志系统和命令行参数。

📄 4. unitree_articulation.h 
将 Unitree 底层 DDS 数据转换为 isaaclab 统一格式，桥接硬件和算法层。

## deploy/robots/g1/config 文件解释
```
deploy/robots/g1/config/
├── config.yaml                         # ← 主配置文件（FSM状态机定义）
└── policy/                             # ← 策略模型目录
    ├── velocity/                       #   速度控制策略
    │   ├── height_flat_v0/             #     版本：带高度控制的平地行走
    │   │   ├── exported/               #       ONNX 模型文件
    │   │   │   └── policy.onnx         #       (1.4 MB)
    │   │   └── params/                 #       部署参数
    │   │       ├── deploy.yaml         #       环境配置
    │   │       └── deploy.yaml.bak_*   #       备份文件
    │   ├── smoke_test_prepare/         #     测试版本
    │   ├── v0/                         #     基础版本
    │   └── v0_keyboard/                #     键盘控制版本
    └── mimic/                          #   动作模仿策略
        └── dance1_subject2/            #     舞蹈动作模仿
            ├── exported/               #       ONNX 模型 + 外部数据
            │   ├── policy.onnx         #       (13.9 KB)
            │   └── policy.onnx.data    #       (967 KB)
            └── params/                 #       部署参数
                ├── dance1_subject2.npz #       参考动作数据 (11.5 MB)
                └── deploy.yaml         #       环境配置
```
1️⃣ config.yaml 
```
作用：定义 FSM 状态机的所有状态、转换规则和参数
```
2️⃣ policy/velocity/height_flat_v0/exported/policy.onnx
```
作用：训练好的强化学习策略网络（ONNX 格式)
```
3️⃣ policy/velocity/height_flat_v0/params/deploy.yaml 
```
作用：定义 RL 环境的详细配置，必须与训练时保持一致
```
## deploy/robots/g1/include文件夹解释
```
deploy/robots/g1/include/
├── Types.h                              # ← 类型定义（G1 专用）
├── G1_HeightArticulation.h              # ← 高度控制机器人抽象层
├── RightArmIKLCMBridge.h                # ← 右臂 IK 控制 LCM 桥接                 
└── lcm_types/                           #   LCM 消息类型定义
    ├── arm_action_lcmt.hpp              #     手臂动作消息
    └── body_control_data_lcmt.hpp       #     身体控制数据消息
```
1️⃣ Types.h
```
作用：为 G1 机器人定义 DDS 通信的类型别名
✅ 使用 G1 专用的 DDS wrapper
✅ 同时支持 Go2（可能是为了兼容或测试）
✅ 通过 using 简化类型名称，提高代码可读性
```
2️⃣ G1_HeightArticulation.h 
```
作用待明确
```
3️⃣ RightArmIKLCMBridge.h 
```
通过 LCM（Lightweight Communications and Marshalling） 接收外部逆运动学（IK）解算结果，控制右臂执行特定任务
```
4️⃣ lcm_types/arm_action_lcmt.hpp 
```
定义 LCM 通信的消息格式，用于传输手臂关节角度目标
```
5️⃣ lcm_types/body_control_data_lcmt.hpp
```
发布机器人当前状态，供外部 IK 求解器使用
```
## deploy/robots/g1/scripts文件夹解释
```
deploy/robots/g1/scripts/
├── build.sh              # ← 编译控制器程序
├── prepare_policy.sh     # ← 准备策略文件（整理训练产物）
└── run_ctrl.sh           # ← 运行控制程序（仿真/实机）
```
## main.cpp
这是整个 G1 机器人控制程序的主入口点
1.系统初始化（参数加载、DDS 通信配置）
2.底层连接（建立与机器人的 DDS 通信）
3.状态机启动（创建并运行 FSM 控制器）

## 实机部署流程
### 数据流
```
┌─────────────────────────────────────────────────────┐
│              实机部署运行流程                         │
└─────────────────────────────────────────────────────┘

1️⃣ 机器人底层状态 (DDS通信)
   ↓
   LowState_t: 关节位置、速度、IMU数据等
   
2️⃣ Articulation 层 (deploy/include/unitree_articulation.h)
   ↓
   将底层数据转换为统一格式
   
3️⃣ ObservationManager (isaaclab/manager/observation_manager.h)
   ↓
   调用观测函数 (isaaclab/envs/mdp/observations/observations.h)
   - base_ang_vel
   - projected_gravity  
   - joint_pos_rel
   - joint_vel_rel
   - last_action
   - velocity_height_commands (来自手柄)
   
   应用：噪声 + 缩放 + 历史窗口
   
4️⃣ OrtRunner (isaaclab/algorithms/algorithms.h)
   ↓
   加载 policy.onnx
   执行 ONNX 推理
   输出归一化动作
   
5️⃣ ActionManager (isaaclab/manager/action_manager.h)
   ↓
   JointPositionAction (isaaclab/envs/mdp/actions/joint_actions.h)
   - 应用 scale
   - 应用 offset
   - 应用 clip
   
6️⃣ 发送到底层电机
   ↓
   lowcmd->motor_cmd[joint_id].q() = processed_action
```
