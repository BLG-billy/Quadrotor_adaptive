# x500 圆轨迹控制实验

本包运行 ROS 2 / PX4 SITL / Gazebo x500 实验，提供纯几何、几何＋L1、
CTBR 和 PX4 内置位置控制四种模式。统一入口自动启动轨迹、控制、飞行管理、
记录四个独立进程，并保存轨迹图和误差 CSV。

当前只有一套默认几何增益生成方法和一套 x500 非线性执行器映射。
已删除 `paper` 增益分支和旧线性映射，不再需要选择历史版本。

## 1. 编译与启动

当前环境为 ROS 2 Humble。PX4 和工作区中的 `px4_msgs` 定义必须匹配。
本地消息版本为 VehicleStatus v4、VehicleLocalPosition v1；节点按
`MESSAGE_VERSION` 自动选择话题后缀。

```bash
cd ~/ws_sensor_combined
source /opt/ros/humble/setup.bash
colcon build --packages-select px4_msgs circle_offboard_py --symlink-install
source install/setup.bash
```

仅修改本包时，可只构建 `circle_offboard_py`。

终端 1，启动 PX4/Gazebo：

```bash
cd ~/PX4-Autopilot
export ROS_DOMAIN_ID=42
PX4_UXRCE_DDS_NS=sim PX4_GZ_WORLD=circle_track make px4_sitl gz_x500
```

若没有本地 `circle_track` world，省略 `PX4_GZ_WORLD=circle_track`。
PX4 控制台中的 `UXRCE_DDS_DOM_ID` 也必须为 42；终端环境变量不会修改飞控参数。
必要时执行 `param set UXRCE_DDS_DOM_ID 42` 并重启 PX4。

终端 2，启动代理：

```bash
export ROS_DOMAIN_ID=42
MicroXRCEAgent udp4 -p 8888
```

打开 QGroundControl 并连接仿真机。当前本地配置可能因缺少地面站连接拒绝解锁。

终端 3，运行实验前准备：

```bash
cd ~/ws_sensor_combined
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
ros2 topic list --no-daemon --spin-time 5
```

应看到 `/sim/fmu/out/vehicle_odometry`、`vehicle_status_v4` 和
`vehicle_local_position_v1`。开始前关闭上一轮控制节点，不同时运行多种控制器。
本包用于隔离的 SITL 命名空间，勿将实验指令接入真机。

## 2. 选择实验模式

以下命令均显式开启自动请求 Offboard/解锁，且明确指定扰动开关。
不需要自动解锁时，删除 `-p enable_auto_arm:=true`，由外部完成模式切换和解锁。
默认 `enable_auto_arm=false`。

### 纯几何控制，无扰动

```bash
ros2 run circle_offboard_py controller_experiment --ros-args \
  -p enable_auto_arm:=true -p enable_l1:=false \
  -p enable_disturbance:=false -p experiment_name:=geometric
```

### 几何＋L1，无扰动

```bash
ros2 run circle_offboard_py controller_experiment_l1 --ros-args \
  -p enable_auto_arm:=true -p enable_disturbance:=false
```

### 相同扰动下比较有无 L1

分别运行，每轮完成返航降落并关闭进程后再开始下一轮：

```bash
# 无 L1，有扰动
ros2 run circle_offboard_py controller_experiment --ros-args \
  -p enable_auto_arm:=true -p enable_disturbance:=true \
  -p experiment_name:=geometric_disturbed

# 有 L1，有扰动
ros2 run circle_offboard_py controller_experiment_l1 --ros-args \
  -p enable_auto_arm:=true -p enable_disturbance:=true \
  -p experiment_name:=l1_disturbed
```

L1 只估计模式：在 L1 命令后加 `-p l1_apply_compensation:=false`。
该模式运行预测/估计/滤波，但不把补偿施加给飞行器。

### CTBR 与 PX4 位置控制

```bash
# 几何外环＋PX4 角速度内环
ros2 run circle_offboard_py controller_experiment_ctbr --ros-args \
  -p enable_auto_arm:=true -p enable_disturbance:=false

# PX4 内置位置控制
ros2 run circle_offboard_py circle_offboard --ros-args \
  -p enable_auto_arm:=true -p experiment_name:=position
```

| 模式 | 发布接口 | 内环分工 |
|---|---|---|
| 几何 / L1 | `vehicle_thrust_setpoint`、`vehicle_torque_setpoint` | 本包计算物理力矩，PX4 分配 |
| CTBR | `vehicle_rates_setpoint` | 本包生成目标机体角速度，PX4 关闭角速度环 |
| 位置 | `trajectory_setpoint` | PX4 执行位置、姿态和角速度控制 |

CTBR 不支持本包的 L1 力矩补偿；位置模式不支持这里的推力通道扰动。
统一入口已经包含记录器，不要再同时运行第二个 `trajectory_monitor`。

## 3. 当前默认参数

### 几何控制增益

默认采用离线搜索得到的候选配置。它尚未完成同条件 Gazebo 飞行对照，
不能视为实际飞行最优值。

| 参数 | 默认值 |
|---|---|
| `position_frequency` | [2.5, 2.5, 2.5] rad/s |
| `position_damping` | 1.0 |
| `attitude_frequency` | [10.0, 10.0, 6.0] rad/s |
| `attitude_damping` | 0.85 |
| `mass` | 2.0643076923 kg |
| `inertia_xx/yy/zz` | 0.0216667 / 0.0216667 / 0.04 kg·m² |
| `gravity` | 9.81 m/s² |

生成关系（逐轴计算，频率单位是 rad/s）：

```text
Kx = m*wn_position²      Kv = 2*zeta_position*m*wn_position
KR = J*wn_attitude²      KOmega = 2*zeta_attitude*J*wn_attitude
```

当前对应：

| 增益 | x / y / z |
|---|---|
| Kx | 12.901923 / 12.901923 / 12.901923 |
| Kv | 10.321538 / 10.321538 / 10.321538 |
| KR | 2.166667 / 2.166667 / 1.440000 |
| KOmega | 0.368333 / 0.368333 / 0.408000 |

仍可显式覆盖 `kx_x/y/z`、`kv_x/y/z`、`kR_x/y/z`、`kOmega_x/y/z`。
直接覆盖优先于对应频率生成值，调参时不要混用而忘记已有覆盖。
控制节点启动时会打印最终生效的四组增益，便于确认命令/YAML 是否覆盖默认值。
CTBR 共享位置环，使用独立 `ctbr_attitude_gain_x/y/z=4/4/3`，不执行几何力矩律。
PX4 位置模式不使用本包这些增益。

### 轨迹与控制限幅

| 参数 | 默认值 | 说明 |
|---|---|---|
| `hover_height` | 1.0 m | 高度，NED 目标 z 为负 |
| `radius` | 1.0 m | 圆半径 |
| `speed` | 0.5 m/s | 匀速段速度 |
| `takeoff_seconds` | 2.0 s | 平滑起飞，包含在悬停阶段内 |
| `hover_seconds` | 5.0 s | 起飞＋悬停总时长 |
| `circle_ramp_seconds` | 2.0 s | 圆周平滑加速时长 |
| `circle_laps` | 2 | 圈数 |
| `yaw` | 0.0 rad | 固定目标航向 |
| `hover_only` | false | true 时只做定点实验 |
| `hover_experiment_seconds` | 16.0 s | 定点实验总时长 |
| `rate_hz` | 100.0 Hz | 几何/L1/CTBR 定时器频率上限 |
| `state_timeout` | 0.1 s | 状态接收过期阈值 |
| `max_commanded_rate` | 3.0 rad/s | 各轴期望角速度限幅 |
| `max_commanded_acceleration` | 15.0 rad/s² | 各轴期望角加速度限幅 |
| `max_thrust` | 0.90 | 各电机归一化输出上限 |
| `max_torque_command` | 0.80 | 归一化力矩指令绝对值上限 |

从首次收到 `flight_state=running`（已解锁、Offboard、指令新鲜）开始计时：
0–2 s 起飞，2–5 s 悬停，5–7 s 加速进入圆周，7–约 31.13 s 匀速段，
随后 `complete` 并请求 RTL。返航、降落具体行为由 PX4 配置决定。
悬停按固定时长结束，不检测高度稳定；圆心为 NED `(-radius, 0)`，起点 `(0,0)`。
起飞假定从 NED 原点开始，实际起飞时长不超过 hover_seconds。
`circle_ramp_seconds=0` 会关闭入口平滑，重新引入速度跳变。

### L1 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `enable_l1` | false；L1 专用入口 true | 启用估计器 |
| `l1_apply_compensation` | true | false 为只估计 |
| `l1_predictor_rates` | [5,5,5,10,10,10] 1/s | 速度三轴、角速度三轴；As 为其负对角阵 |
| `l1_cutoff` | [10,3,3,3] rad/s | 推力、Mx、My、Mz 一级低通 |
| `l1_enable_second_filter` | false | 可选的第二级力矩滤波 |
| `l1_moment_cutoff_second` | [15,15,15] rad/s | 仅第二级开启时使用 |
| `l1_compensation_limits` | [5,0.4,0.4,0.15] | 补偿上限，N、N·m、N·m、N·m |
| `l1_max_interval` | 0.05 s | 最大估计间隔 |

推力截止频率默认由 5 提高为 10 rad/s，以减少正弦推力扰动补偿的相位滞后；
三轴力矩仍为 3 rad/s。该值需通过同条件飞行对照验证，不保证所有工况更优。
可用 `-p 'l1_cutoff:=[5.0, 3.0, 3.0, 3.0]'` 回到此前配置；
命令行/YAML 显式覆盖优先于默认值。比较时重点查看高度 RMS、峰值和输出波动。

要求 `max(l1_predictor_rates)*l1_max_interval < 1`。
补偿限幅不回写滤波器内部状态。L1 仅在 running 且有新有效观测时学习；
状态/参考过期、运行切换、采样时间回退、超长间隔或数值异常会重置历史。
未 running 时只发布基础控制。running 不代表已经离地。

### 扰动开关

`enable_disturbance` 未指定时跟随启动时的 `enable_l1`：纯几何/CTBR 默认关闭，
L1（含只估计模式）默认开启。显式 true/false 独立覆盖，修改后需重启实验。

推力扰动单位为 N，t 为首次 running 后的时间：

```text
d(t) = 0                                   t < 2 s
       2*sin(2*pi*t) + 2*sin(pi*t)       t >= 2 s
```

扰动在基础控制＋L1 补偿之后、执行器映射之前加入，正值增加沿机体 -z 的推力，
不注入力矩通道；waiting 时不注入，恢复 running 不重置起始时钟。
当前两个幅值均为 2 N（峰值约 3.52 N）；论文原幅值各为 0.6 N。
当前为针对 x500 放大的扰动，不能当成论文原始幅值。

论文正弦扰动是在悬停条件下验证；定点模式示例：

```bash
ros2 run circle_offboard_py controller_experiment_l1 --ros-args \
  -p enable_auto_arm:=true -p hover_only:=true \
  -p hover_experiment_seconds:=16.0 -p enable_disturbance:=true
```

只悬停不进入 circle，因此现有记录器不会输出有效圆周 RMS；查看位置 CSV
和控制/L1 日志分析悬停响应。

## 4. 参数覆盖与旧配置迁移

参数使用 `--ros-args -p 名称:=值`，统一入口传给各子进程。
YAML 使用 `/**` 或实际节点名。开关和增益在启动时读取，不是运行时动态调参。

当前无需指定增益配置名称。请从旧命令/YAML 中删除：

```text
gains_profile
actuator_mapping
hover_thrust
torque_scale_x / torque_scale_y / torque_scale_z
```

这些参数已不再使用；保留旧 ROS 参数覆盖可能不会报错，但不会选择旧实现。
如希望使用当前默认增益，同时删除原有 kx/kv/kR/kOmega 的显式覆盖。
历史结果与 [离线搜索结果](results/gain_search_20261024.json) 保留用于追溯，
它们不是运行时参数源。

## 5. 结果保存与误差口径

| 参数 | 默认值 |
|---|---|
| `fmu_namespace` | `sim`，话题前缀 `/sim/fmu` |
| `experiment_name` | geometric；L1/CTBR 专用入口分别为 geometric_l1/ctbr |
| `results_dir` | `~/ws_sensor_combined/src/circle_offboard_py/results` |
| `show_plot` | true；无显示环境自动禁用窗口 |
| `save_control_diagnostics` | true，纯几何与 L1 生效 |

`show_plot:=false` 只关闭窗口，不影响保存。文件名采用北京时间和微秒时间戳：

| 文件 | 内容 |
|---|---|
| `<name>_<timestamp>.png/.csv` | 全程位置、对齐参考及圆周轨迹/水平误差 |
| `*_control_diagnostics.csv` | 几何力矩五分项、期望导数、误差、限幅和发送指令 |
| `*_l1_diagnostics.csv` | 启用 L1 时的状态预测、扰动估计与补偿 |

各进程独立生成文件时间戳，可能略有差异。complete 时自动保存轨迹，
Ctrl+C 也保存已有有效数据；样本不足不生成空结果。保存后程序继续运行，
Ctrl+C 统一关闭。诊断每 50 行刷新，正常关闭时关闭文件。

只统计悬停后的 circle 阶段，包含启动加速段：

```text
horizontal_error = sqrt((x-x_ref)² + (y-y_ref)²)
RMS = sqrt(mean(horizontal_error²))
```

这是水平误差，不是三维误差。以记录器首次收到 circle 状态的接收时间为边界，
并按接收时间插值参考；受通信/调度延迟影响。不使用 0.1 m/s 速度阈值。
CSV 的 time_s 从记录器启动开始，包含解锁前等待；图从首个有效圆周样本计零。
悬停和参考范围外的误差为 NaN、circle_phase=0，不计入 RMS。

主要诊断：

- `omega_c_*`、`omega_c_dot_*`、`eR_*`、`eOmega_*`：期望导数和跟踪误差。
- `attitude_*`、`rate_*`、`gyro_*`、`transport_*`、`acceleration_*`：基础力矩分项。
- `torque_fraction`：可实现力矩比例，小于 1 表示统一缩小力矩。
- `requested_*`、`sent_*`、`normalized_*`：包含人工扰动的请求、重建物理输入、发送归一化值。
- `limited_*` / `saturated_*`：请求与重建值是否存在超容差差异。
- `injected_thrust_N`：请求注入的正弦扰动。
- L1 的 `predictor_f/mx/my/mz`：排除人工扰动、通过同一限幅映射的预测输入。
- L1 的 `sigma_*`、`adaptive_raw_*`、`adaptive_applied_*`、`reset_count`：估计、补偿和重置情况。

## 6. 代码结构与控制链

| 文件 | 职责 |
|---|---|
| [controller_experiment.py](circle_offboard_py/controller_experiment.py) | 统一子进程启动与关闭 |
| [node_runner.py](circle_offboard_py/node_runner.py) | 单节点运行与清理 |
| [flight_manager_node.py](circle_offboard_py/flight_manager_node.py) | 心跳、解锁、运行状态和 RTL |
| [trajectory_node.py](circle_offboard_py/trajectory_node.py) | 阶段计时和参考发布 |
| [trajectory_reference.py](circle_offboard_py/trajectory_reference.py) | 解析轨迹和完整参考包校验 |
| [geometric_controller_node.py](circle_offboard_py/geometric_controller_node.py) | 唯一默认增益、状态接收、调度、发布与诊断 |
| [geometric_control.py](circle_offboard_py/geometric_control.py) | SE(3) 公式及解析姿态导数 |
| [actuator_mapping.py](circle_offboard_py/actuator_mapping.py) | x500 非线性反解与电机可行性约束 |
| [l1_adaptive_control.py](circle_offboard_py/l1_adaptive_control.py) | L1 预测、扰动估计和滤波 |
| [l1_geometric_controller_node.py](circle_offboard_py/l1_geometric_controller_node.py) | 可选 L1 门控、补偿和日志 |
| [injected_disturbance.py](circle_offboard_py/injected_disturbance.py) | 人工推力扰动公式 |
| [ctbr/ctbr_control.py](circle_offboard_py/ctbr/ctbr_control.py) | 姿态误差到机体角速度 |
| [ctbr/ctbr_controller_node.py](circle_offboard_py/ctbr/ctbr_controller_node.py) | CTBR 输出接口 |
| [position_controller_node.py](circle_offboard_py/position_controller_node.py) | PX4 位置参考转发 |
| [recorder_node.py](circle_offboard_py/recorder_node.py) | 绘图与记录 |
| [trajectory_results.py](circle_offboard_py/trajectory_results.py) | 对齐和误差统计 |

常规参考 `/circle/reference_trajectory` 使用 TrajectorySetpoint；
几何/L1/CTBR 优先使用 `/circle/reference_high_order`，类型 Float64MultiArray：

```text
[timestamp_us, p(3), v(3), a(3), jerk(3), snap(3), yaw, yaw_rate, yaw_accel]
```

同一包内各阶导数属于同一时刻。仅常规参考输入时，缺少 snap/yaw_accel 按零处理。
`/circle/flight_state` 为 running/waiting；`/circle/experiment_state` 为 hover/circle/complete。

控制器使用 NED/FRD，解析计算 A_dot/A_ddot、Rc_dot/Rc_ddot 和期望角速度/
角加速度，不再两次数值差分。角速度饱和分量的角加速度前馈置零。
几何/L1/CTBR 仅按新里程计更新，优先用采样时间间隔；位置模式直接转发参考。

总推力＋力矩先经过电机可行性约束与非线性反解，再编码为 PX4 归一化输入。
不可实现时先裁剪总推力，再统一缩小力矩。映射要求标准 x500 的转子顺序、
位置和归一化 RPY 分配，THR_MDL_FAC=0、SIM_GZ_EC_MIN1..4=150、MAX1..4=1000。
模型系数为 kf=8.54858e-6、力矩/推力比 0.016、机臂坐标绝对值 0.174 m。
节点不会自动检查或修改飞控参数。CTBR 只做等转子总推力反解，最终余量由 PX4 处理。

发送后保存 float32 指令及其等效稳态物理 wrench。L1 预测输入单独排除
人工扰动，使其作为未知输入被估计；饱和时实际扰动效应可能不同于请求正弦值。
这不是测得的转子力，不含电机滞后或未知分配误差。
L1 是变步长工程实现，不直接继承论文固定采样稳定性结论。

## 7. 检查与验证

没有飞控话题时检查 Agent、DDS 域、命名空间、消息版本和 PX4
`uxrce_dds_client status`。有状态但无动作时查看飞行管理每 5 秒的等待提示。
异常高度/力矩应核对模型参数、torque_fraction 和 PX4 分配器状态。

```bash
ros2 topic hz /sim/fmu/out/vehicle_odometry
```

功能测试，不启动 PX4/Gazebo：

```bash
cd ~/ws_sensor_combined
source /opt/ros/humble/setup.bash
source install/setup.bash
PYTHONPATH="$PWD/src/circle_offboard_py:$PYTHONPATH" /usr/bin/python3 -m pytest \
  src/circle_offboard_py/test/test_analytic_control.py \
  src/circle_offboard_py/test/test_geometric_dynamics.py \
  src/circle_offboard_py/test/test_ctbr_control.py \
  src/circle_offboard_py/test/test_l1_adaptive_control.py \
  src/circle_offboard_py/test/test_l1_node.py \
  src/circle_offboard_py/test/test_injected_disturbance.py \
  src/circle_offboard_py/test/test_modular_experiment.py -q
```

覆盖解析公式、轨迹连续性、非线性分配、采样时序、L1、扰动门控、独立动力学
及五种实验配置的节点通信。通信测试使用 DDS 域 187 / refactor_test 命名空间。
这些测试不代替实际 Gazebo 飞行验证；ROS 标称 100 Hz 不等于飞控内部 400 Hz，
也未消除通信延迟和模型误差。

参考：[L1Quad](https://sheng-cheng.github.io/research/l1quad)、
[官方实现](https://github.com/sigma-pi/L1Quad)。

### 更新频率诊断

几何与 L1 共享一个新观测序号和控制期限调度。定时器维持标称节拍；如果
定时器到期时尚无新观测，随后到达的有效里程计可立即完成这次到期更新。
不重复使用旧状态进行自适应，也不在积压后补算多次。输入不足 100 Hz 或回调
超时仍会限制实际频率，不保证硬实时。

控制节点每 5 秒打印实际发布频率、有效里程计接收频率、回调耗时、峰值和超周期次数。
控制诊断 CSV 新增 previous_callback_ms（上一周期耗时，避免把本行写入时间
漏算）、callback_peak_ms、control_overruns、recent_control_hz、recent_odometry_hz。
actual dt 仍使用采样时间戳。L1 固定惯量逆矩阵在初始化缓存，输入矩阵使用
等价的分块解析逆解，不改变估计公式、增益或滤波截止频率。

历史 50–55 Hz 不能仅由旧 CSV 确定根因：其未记录输入接收率和回调耗时。
隔离 ROS 测试改动前已经可达约 100 Hz；本次消除重复门控并覆盖定时器/观测
错相情形，实际 Gazebo 是否恢复需下一轮由新诊断确认。
