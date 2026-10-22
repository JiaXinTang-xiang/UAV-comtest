# 模块 C 无人机仿真训练

工程目录：`/home/jiaxintang/桌面/bao`。使用 PX4 v1.14.3、Gazebo Classic 11、ROS2 Humble、原生 Micro XRCE-DDS 接口。系统库仍在 `/usr`、`/opt/ros`；PX4、Agent、消息包和工程源码在本目录。

**最新功能、左右路线坐标、完整启动命令及成功记录：** [仿真功能与双路线启动说明](docs/仿真功能与双路线启动说明.md)。

**统一仓库与依赖保存方式：** [仓库与依赖管理](docs/仓库与依赖管理.md)。本次截图和终端附件保存在 `docs/现场记录/2026-10-22/`。

**移植到其他虚拟机：** [虚拟机迁移与依赖安装](docs/虚拟机迁移与依赖安装.md)，包含系统版本、依赖安装、目录打包、重新编译与启动命令。

## 正常训练流程

每个命令在独立终端运行并保持打开。先结束旧的悬停或巡航节点；它们共用控制话题，不能同时运行。重新训练一趟前重启仿真和控制节点。

**终端 1：通信 Agent**（已运行则保留）

```bash
cd /home/jiaxintang/桌面/bao
./scripts/run_xrce_agent.sh
```

**终端 2：比赛场景和 PX4**

```bash
cd /home/jiaxintang/桌面/bao
./scripts/run_px4_competition.sh
```

脚本会清理旧的 PX4/Gazebo 并先编译；传感器无人机随世界文件直接加载，等待 MAVLink TCP 4560 就绪后启动 PX4。服务端和 GUI 日志在 `logs/simulation/`。GUI 无法操作时先看该目录日志。无人机初始位置为 `(5,9)`。

**终端 3：传感器和区域巡航节点**

```bash
cd /home/jiaxintang/桌面/bao
./scripts/run_px4_regions.sh route:=right
```

等待自检报告 `READY`，飞行状态 `WAITING -> READY`。节点不会自动解锁。

**终端 4：开始巡航**

```bash
cd /home/jiaxintang/桌面/bao
./scripts/start_mission_when_ready.sh
```

流程：系统自检 → Offboard 心跳预发送 → 切换模式 → 解锁 → 2.5 m 起飞 → 区域 2→3→4→5 → 靶标坐标上方减速对准 → PX4 自动降落 → 确认接地及上锁 → COMPLETE。所有完成判定来自飞控状态；服务返回成功表示请求已受理。

紧急落地：

```bash
ros2 service call /uav/flight/land std_srvs/srv/Trigger '{}'
```

确认落地上锁后 Ctrl+C 关闭任务终端，再关闭仿真；飞行中直接关闭控制节点会触发 PX4 自身的 Offboard 丢失保护。

### 只练习传感器和悬停

终端 3 改用 `./scripts/run_px4_sensors.sh`，然后调用：

```bash
ros2 service call /uav/flight/start std_srvs/srv/Trigger '{}'
```

该模式自检通过后起飞，悬停 5 秒并自动降落。不要同时运行区域巡航节点。

## 数据来源与自检

| 数据 | 实际来源 | ROS2 话题 |
|---|---|---|
| 电量 | PX4 battery_simulator 电池模型，解锁时随时间消耗 | `/fmu/out/battery_status` → `/uav/battery` |
| IMU | Gazebo IMU → MAVLink → PX4 | `/fmu/out/sensor_combined` → `/uav/sensors/imu` |
| 光流 | 向下 64×64 灰度相机，OpenCV 图像特征跟踪 | `/fmu/out/sensor_optical_flow` → `/uav/sensors/optical_flow_raw` |
| 向下测距 | Gazebo 单束 ray 与地面碰撞距离 | `/fmu/out/distance_sensor` → `/uav/sensors/downward_range` |
| 水平雷达 | Gazebo 360 束 ray，15 Hz，0.12–15 m | `/uav/lidar/scan` → `/uav/sensors/lidar` |
| 位姿和速度 | PX4 EKF 估计，NED 转换到 Gazebo ENU 地图 | `/uav/pose`、`/uav/velocity` |
| 独立真值 | Gazebo p3d，单独评分记录，不输入控制 | `/uav/ground_truth/odom` |

自检检查数据数值、源时间戳更新、接收新鲜度、电池连接/电压/剩余量、光流积分时间/质量、测距朝向/质量、雷达扫描结构。单个无效光流帧被拒绝，不更新有效数据时间；连续无效或断流超过 1 秒将阻止起飞或触发落地，电池超时为 2.5 秒。不要将无效质量改成通过。

查看状态：

```bash
source /opt/ros/humble/setup.bash
source /home/jiaxintang/桌面/bao/px4_msgs_overlay_ws/install/setup.bash
ros2 topic echo /uav/preflight/report
# 另一个终端：
ros2 topic echo /uav/mission/state
```

记录原始传感器、飞行状态、估计位置和独立真值：

```bash
python3 /home/jiaxintang/桌面/bao/scripts/check_px4_sensors.py \
  /home/jiaxintang/桌面/bao/logs/flight/manual_sensor_check.json 180
```

光流相机画面：`/uav/flow/camera/image_raw`。训练地面加了透明特征纹理，方便相机看到纹理，墙体碰撞几何没有修改。

## 规划与安全

`uav_competition_ws/src/uav_competition/config/mission.yaml` 中的 `route_profiles` 配置左右路线的区域目标与中转点；运行 `./scripts/run_px4_regions.sh route:=right` 或 `route:=left` 选择。默认右路线。旧 `mission.route` 留作图示参考。

规划器从 world 的墙体碰撞盒生成保持机体净距的 A* 路径；水平雷达识别地图外的新障碍，检查前方航段并保持当前位置，在工作线程重新规划。规划有时间/重试上限，无解或近距危险时降落。控制节点持续发送 Offboard 心跳并限制参考速度。规划路径发布到 `/uav/mission/path`，触发原因和新路径写入任务 JSON。

新增规划与安全功能需要用完整任务日志验收。当前训练地图提供两次独立完整任务：第一次选择右路线，第二次选择左路线。左路线使用训练地图外侧窄通道，配置为 0.35 m 规划净距和 0.25 m/s 慢速；全局地理围栏、走廊偏离、超时、传感器断流、激光雷达近障和无安全路径自动降落保护始终开启。该净距是本训练地图和 Iris 模型的工程假设，正式赛场模型或墙体变化后需重新配置。

开发阶段可以用 `enable_tests:=true` 做故障注入，但比赛操作不需要逐项调用这些测试服务。正常任务启动后安全策略由节点自动执行。

安全故障注入默认关闭，只有 `enable_tests:=true` 才可使用。详细操作、验收预期和题目剩余项见 [功能验证流程](docs/功能验证流程.md)。精度验收按当前约定排在功能之后。

`config/px4-rc.params` 设置真实测距安装偏移、平地测距高度参考和 1200 秒电量消耗周期。电量来自 PX4 仿真模型，不代表真实电化学电池；本轮没有校验低电量注入保护。当前光流有数据及自检，导航位置仍主要由 PX4 默认 GPS/IMU 估计，尚未完成无 GPS 光流导航验收。

## 构建及测试

```bash
cd /home/jiaxintang/桌面/bao
./scripts/build_and_source.sh
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=uav_competition_ws/src/uav_competition \
  python3 -m pytest -q uav_competition_ws/src/uav_competition/test
```

PX4 DDS 发布列表有电量、测距、原始光流、指令回执、接地状态，修改后须重新 `make px4_sitl_default`。Gazebo 光流 OpenCV 插件修正了静止零方差与特征重分配造成的虚假位移，源码补丁归档在 `patches/`，重装上游后需恢复。

`simulation.launch.py use_mock:=true` 是旧的运动学练习，使用明确标记的 MOCK 自检、固定电量和速度代理光流，不能作为上述 PX4 传感器或物理飞行验收。正常训练使用本页 PX4 脚本。

## 归档

`logs/flight/` 保存 CSV 飞控采样、任务结果 JSON、传感器验证记录；PX4 ULog 在 `PX4-Autopilot/build/px4_sitl_default/rootfs/log/`。比赛还需按赛方要求开启 EV 录屏并保存视频、代码和参数；当前尚未实现自动录屏归档。
