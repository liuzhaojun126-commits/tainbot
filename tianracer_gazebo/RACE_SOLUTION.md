# TianRacer 赛道竞速最终方案（5 圈最快、稳定无碰撞）

## 结论

- **最终竞速方案：纯追踪（Pure Pursuit）+ 按曲率限速**，控制器 `scripts/pp_race.py`，参考线 `scripts/waypoint_race/tianracer_racetrack_centerline.csv`。
- **真实 Gazebo headless 实测（仿真时间 sim time）连跑两遍，每遍 5 圈：**

| 遍 | 各圈用时 (s) | 5 圈合计 |
|---|---|---|
| 1 | 17.95 / 17.90 / 17.95 / 17.85 / 17.85 | **89.50 s** |
| 2 | 17.95 / 17.90 / 17.95 / 17.95 / 17.90 | **89.65 s** |

- 圈速抖动约 0.05 s，**全程 0 碰撞**，单圈约 18 s。取最短 **89.50 s**，与基线一致报 **约 89.6 s / 5 圈**。
- move_base + TEB 在本赛道**不用于竞速**（原因见文末），仅在其参数文件中修正阿克曼运动学常量。

## 写死参数（已作为 `pp_race.py` 默认值）

| 参数 | 值 | 含义 |
|---|---|---|
| vmax | 6.0 m/s | 直道最高速度 |
| alat | 8.0 m/s² | 横向加速度限制，弯道按 v=√(alat/κ) 限速 |
| acc | 6.0 m/s² | 最大前进加速度 |
| dec | 9.0 m/s² | 最大制动减速度 |
| k_ld / ld0 | 0.22 / 0.45 | 前视距离系数与最小前视 |
| wheelbase | 0.26 m | 阿克曼轴距 |
| laps | 5 | 圈数 |

> 实测 ALAT>8 会在发卡弯高速冲出贴墙，故锁定 ALAT=8。

## 一键复现（无头仿真）

先 source ROS 环境与本工作区，然后一条命令跑 5 圈：

```bash
roslaunch tianracer_gazebo pp_race_headless.launch
```

该 launch 启动 headless gzserver + 底盘控制，并运行 `pp_race.py`（默认参数即上表）。
也可显式传参：

```bash
roslaunch tianracer_gazebo pp_race_headless.launch vmax:=6.0 alat:=8.0 acc:=6.0 dec:=9.0 laps:=5
```

或分两步：

```bash
roslaunch tianracer_gazebo pp_headless.launch   # 只起 gzserver + 底盘
rosrun tianracer_gazebo pp_race.py              # 另开终端运行竞速节点
```

跑完节点会打印 `PP DONE laps=5 times=[...]`，列表之和即 5 圈正式用时（`total_from_t0` 含启动对位时间，不计入成绩）。

## 为什么不用 TEB（move_base + TebLocalPlannerROS）

对标准 move_base + TEB 用 dynamic_reconfigure 单变量扫描约 4.5 小时（速度、dt_ref/轨迹密度、obstacle/膨胀半径、viapoints、footprint，乃至一个直接喂赛车线的 C++ 全局规划器）：

- **任何固定参数都无法完成 1 圈**，最好的一组（global/local 膨胀 0.25/0.02）仅沿赛道无碰撞走到约 60%（第 21 个目标点），随后报 `trajectory is not feasible. Resetting planner...`（累计 137 次）并在恢复行为耗尽后 Abort。
- 根因是几何层面：赛道含 8 个内宽仅约 0.21 m 的发卡弯，而车半宽约 0.05~0.06 m，固定安全裕度在“窄口进得去”与“直道不贴墙”之间顾此失彼；降速、加密轨迹均无效。TEB 是避障导航框架，不适合这种窄赛道极限竞速。

本次仅在 `config/teb/teb_local_planner_params.yaml` 修正两个阿克曼运动学常量（与竞速性能无关，属正确性修复）：

- `max_vel_theta`：6.0 → **0.6**（cmd_angle 模式下该字段即最大前轮舵角 rad，应为舵限 0.6）。
- `min_turning_radius`：0.5 → **0.38**（= 轴距 / tan(最大舵角) = 0.26 / tan(0.6) ≈ 0.38 m）。

> 扫描中跑过最远的那组膨胀参数（0.25/0.02）跑不完 1 圈，**未写死**。
