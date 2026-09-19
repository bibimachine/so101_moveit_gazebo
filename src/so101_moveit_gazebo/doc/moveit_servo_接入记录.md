# MoveIt Servo 接入记录(SO101 + Gazebo)

> 日期:2026-09-18。环境:ROS 2 Humble,moveit2 源码构建于 `~/ws_moveit2`(humble 分支,moveit_servo 2.5.9)。
> 配套文件:`moveit_config/servo_parameters.yaml`、`launch/so101_moveit_servo.launch.py`。
> launch 用 `MoveItConfigsBuilder('so101', package_name='so101_moveit_gazebo')` 构建参数
> (本包非标准 `*_moveit_config` 布局, 需显式传 package_name 及每个文件的相对路径;
> `moveit_config/` 目录已在 CMakeLists 中随包安装)。

## 1. 启动顺序

```bash
# 终端1: Gazebo 仿真(控制器 / joint_states / TF)
ros2 launch so101_moveit_gazebo so101_gazebo.launch.py

# 终端2: 手柄遥控链(xbox_reader -> GamepadState -> ee_teleop -> TwistStamped)
ros2 launch ee_teleop ee_teleop.launch.py

# 终端3: Servo(自动调用 /servo_node/start_servo)
ros2 launch so101_moveit_gazebo so101_moveit_servo.launch.py
```

启动后可观察:`ros2 topic echo /servo_node/status`(0=静止,1=伺服中,3=碰撞减速,4=碰撞急停)。

## 2. 全链路对账

```
Xbox 手柄
  └─ joy_node (/joy, sensor_msgs/Joy)
       └─ xbox_reader  joy_reader (/gamepad/state, gamepad_msgs/GamepadState, 语义化+归一化)
            └─ ee_teleop  ee_teleop (/ee_teleop/twist, geometry_msgs/TwistStamped,
            │     死区+EMA平滑+任务空间加速度限幅+看门狗, vx/vy/vz, frame_id=base_link,
            │     摇杆向量经 control_yaw 旋转)
            │       └─ servo_node (/so101_arm_controller/joint_trajectory, 100Hz JointTrajectory,
            │             雅可比求逆IK + 关节限幅 + 奇异点/碰撞缩放 + Butterworth滤波)
            │              └─ so101_arm_controller (joint_trajectory_controller, position 接口)
            │                   └─ Gazebo (gazebo_ros2_control)
            ├─ ee_teleop 手腕支路: Y 键 -> /servo_node/delta_joint_cmds
            │     (control_msgs/JointJog, 仅 wrist_roll 单关节; Servo 2.5.9 双通道互斥、
            │      Twist 优先, 约定不平移与转腕同时操作)
            └─ ee_teleop 夹爪支路: X 键 -> /so101_gripper_controller/commands
                  (Float64MultiArray, effort_controllers/JointGroupEffortController, 与 Servo 无关)
```

| 对接点 | 上游 | 下游 | 核对结论 |
|---|---|---|---|
| 速度量纲 | ee_teleop 输出物理值 m/s、rad/s(`v_max_xy=0.1` 等) | `command_in_type: "speed_units"` | ✅ speed_units 模式 Servo 原样使用,不做二次缩放 |
| 命令话题 | `/ee_teleop/twist` | `cartesian_command_in_topic: /ee_teleop/twist` | ✅ 直接订阅,无中间节点 |
| 命令坐标系 | ee_teleop `base_frame: base_link`(写入 Twist header) | `robot_link_command_frame: base_link`;Servo 按 header.frame_id 经 TF 变换到 planning_frame | ✅ 一致;world→base_link TF 由 URDF `world_joint` 提供 |
| 规划坐标系 | — | `planning_frame: world` | ✅ TF 树中存在 |
| 被控点 | — | `ee_frame_name: gripper_frame_link`(夹爪工具帧) | ✅ URDF 中存在该 link |
| 命令保活 | ee_teleop 30Hz 定时发布;看门狗超时自动发全零 | `incoming_command_timeout: 0.1` | ✅ 持续刷零速,不会误触发超时停止 |
| 输出话题 | — | `command_out_topic: /so101_arm_controller/joint_trajectory` | ✅ 与 Gazebo 侧 `so101_arm_controller`(JTC,100Hz)一致 |
| 输出类型 | — | `command_out_type: trajectory_msgs/JointTrajectory` | ✅ JTC 收 position+velocity 字段(`publish_joint_positions/velocities: true`) |
| 夹爪 | X 键 → `/so101_gripper_controller/commands` | effort 控制器,独立于 Servo | ✅ 无冲突 |
| Servo 启停 | — | 2.5.9 启动后暂停态,launch 内 TimerAction 3s 后调 `/servo_node/start_servo` | ✅ launch 已带 |
| 关节限位 | URDF 限位 + `joint_limit_margin: 0.1` | Servo 关节空间限幅(2.5.9 从 RobotModel 读,无需额外挂 joint_limits.yaml) | ✅ 与 moveit_config/joint_limits.yaml 中数值一致 |
| 速度限幅 | ee_teleop 任务空间 EMA+加速度限幅 | Servo 关节空间限幅 | ✅ 双保险,不冲突 |

## 3. 手柄映射(ee_teleop 现状)

| 输入 | 动作 |
|---|---|
| 左摇杆 X/Y | 地面平行系下 vy / vx（`control_yaw` 定义"前"相对 base +X 的偏航，深浅→速度 0~0.1 m/s） |
| A / B 键 | 末端 vz 升 / 降（0.1 m/s，base 竖直时即垂直地面） |
| Y 键 | wrist_roll 单关节旋转 ±1.5 rad/s（JointJog 通道，纯单关节不过雅可比；Servo 双通道互斥、Twist 优先，约定不平移与转腕同时操作） |
| X 键 | 夹爪开/合切换(上升沿触发) |

## 4. 版本相关备忘(main 分支 vs 本地 2.5.9)

- `pose_command_in_topic`(PoseStamped 输入)仅 main 分支有;2.5.9 若需位姿跟踪要用 C++ `PoseTracking` API。
- main 分支的 `servo_parameters.yaml` 是 generate_parameter_library 构建期声明文件,不能直接加载;本地运行时是平铺 key,由 launch 包一层 `{"moveit_servo": yaml}` 展开(与官方 demo 相同)。
- main 删除了 `planning_frame` / `ee_frame_name` / `robot_link_command_frame` / `use_gazebo` / `low_latency_mode` / `num_outgoing_halt_msgs_to_publish`,改用自动推导或新参数;升级 moveit2 时需对照迁移。
- `joint_limit_margin`(单值)在 main 中变为数组 `joint_limit_margins`。

## 5. 已知待办/注意

- `moveit_config/moveit_controllers.yaml` 里控制器名(`arm_controller`)与 Gazebo 实际名(`so101_arm_controller`)不一致,不影响 Servo(Servo 直连控制器话题);但若以后启动 move_group 需统一命名。
- Servo 只做过单关节限位/奇异点/碰撞的**减速急停**,不会绕行;有障碍场景应 move_group 规划接近、Servo 只做末段对齐。
- `servo_node` 话题/服务均在根命名空间;若改为有命名空间的启动需同步改 `~/delta_joint_cmds` 等话题。
- URDF `<limit velocity="10">` 与 joint_limits.yaml `max_velocity: 2.0` 不一致;2.5.9 Servo 读 URDF,升级或启用 move_group 后生效值会变,建议统一。

## 6. 问题记录与定案(2026-09-18)

### 问题原因

1. **位置控制的固有机制**: gazebo_ros2_control 的位置控制是每 10ms(CM 更新周期)用 `SetPosition` 把关节**硬传送**到命令值;两次传送之间重力把关节拉偏(~0.001 rad 级),下一拍拉回,形成可见的锯齿抖动。且 JTC **inactive 时**(position 接口 unclaimed)只剩 `SetVelocity(0)` 纯阻尼、不约束位置,手臂随重力下坠——调试期观察到的所有"下坠"均发生在 JTC inactive 窗口。
2. **Servo 的活命令模式**: `incoming_command_timeout` 内收到新命令即"活命令",输出 = 当前位形 + 命令位移;ee_teleop 零 Twist 时输出 = 当前位形回读,形成"Servo→JTC→机械臂→joint_states→Servo"回读环路。命令过期则 halt,发若干条刹车消息后静默。
3. **KDL 近似解吃掉平移指令**: SO101 是 5 自由度臂,`kinematics.yaml` 配 KDL 位置 IK 解 6 维位姿是超定的;servo_calcs.cpp 的 IK 调用带 `return_approximate_solution=true`,KDL 返回勉强凑姿态的近似解 → `delta_theta` 极小**且不随命令幅度变化**(实测 0.05 与 0.5 m/s 输出相同),表现就是笛卡尔移动整体失效、shoulder_pan 似乎永远不转,而 JointJog 通道(不过 IK)完全正常。
4. **6DOF 奇异阈值不适用于 5DOF 臂**: 6×5 雅可比条件数天然偏高,默认 `lower=17/hard=30` 使手臂几乎永远处于减速/急停区间。

### 解决方式

- **注释掉 `moveit_config/kinematics.yaml` 的 so101_arm 求解器**(文件内容改为 `{}` 防止 launch 拿到 None):Servo 日志出现 "No kinematics solver instantiated ... Will use inverse Jacobian" 即回退成功。代价:平移时末端姿态不锁(欠定自由度漂移),遥控可接受。
- **奇异阈值改为 `lower=100 / hard=800`**(servo_parameters.yaml),按 5DOF 正常位形标定:工作区全速、接近真伸直才急停。
- **加阻尼缓解抖动**: `so101_description.xacro` 5 个 arm 关节加 `<dynamics damping="1.0"/>`,压低传送窗口内的下坠速度。对 MoveIt/运动学零影响;嫌手柄"肉"可降到 0.3~0.5。
- **操作纪律**: 机械臂异常先 `ros2 control list_controllers` 确认 `so101_arm_controller` active;动没动只信 `/joint_states` 的 position。全零位形报奇异点减速、近限位报 halt 均属正常保护。

### 时间戳教训(CLI 测试时用)

- CLI 发 Twist 必须 `stamp: now`,否则 stamp=0 被 Servo 静默丢弃;CLI 用系统时间、与仿真时钟不同域,测完发一条全零 Twist 刹车。ee_teleop 已加 `use_sim_time: True`,与 Servo 同域。
- 手动发 JTC 轨迹**不要带 stamp**(=0 表示立即执行;`stamp: now` 填系统时间,会被当成十几亿秒后才开始)。
- xacro/URDF 注释里**不能有 ASCII `: `(冒号+空格)**——robot_description 经 rcl 命令行参数解析,会报 `Couldn't parse parameter override rule` 导致插件初始化失败;写完用 `grep ': ' xx.urdf` 自查。
