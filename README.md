# UAV Module C 训练仿真

本目录保留了早期草稿文件，并新增 `uav_competition_ws` 作为可构建的 ROS 2 工程。当前方案已经按确认内容设计：

- PX4 SITL（真实飞控接入目标）
- Gazebo Classic 11
- ROS 2 Humble
- PX4 ROS 2 原生接口（`px4_msgs` / Micro XRCE-DDS）
- IMU、光流话题、激光雷达、电量遥测
- 训练地图墙高暂按 3 m
- 区域和航线坐标集中在 YAML，可随现场参数修改

## 目前可直接运行的训练模式

电脑当前还没有 PX4、`px4_msgs` 和 Micro XRCE-DDS Agent，所以工程提供 `use_mock:=true` 的运动学回退模式，用来先学习完整流程：场景加载、自检、2.5 m 起飞、区域 2→3→4→5、降落和上锁状态逻辑。它不是 PX4 物理飞行验证，真实飞行动力学要在 PX4 SITL 接入后验证。

```bash
cd /home/jiaxintang/桌面/bao
./scripts/build_and_source.sh
source uav_competition_ws/install/setup.bash
ros2 launch uav_competition simulation.launch.py use_mock:=true
```

选择左侧训练路线：

```bash
ros2 launch uav_competition mission.launch.py route:=left
```

每次启动前确认没有遗留 Gazebo 进程；若提示 11345 端口占用，先关闭旧的 Gazebo 窗口或结束旧的 `gzserver/gzclient` 进程。

## 目录说明

- `config/mission.yaml`：训练坐标、路线、2.5 m 高度、安全边界和 3 m 墙高假设
- `worlds/competition_course.world`：按地图重建的 10 m×10 m 训练场景
- `preflight_check.py`：IMU、光流、激光雷达、位姿、速度、电量自检
- `sensor_adapter.py`：统一 Gazebo/PX4 传感器话题
- `mission_controller.py`：起飞、悬停、区域巡航、降落状态机
- `safety_manager.py`：围栏、超高/超低、雷达近障碍和传感器超时保护
- `px4_offboard_node.py`：PX4 原生 ROS 2 Offboard 接口占位与命令转换
- `scripts/install_px4_sitl.sh`：PX4 安装脚本，需确认网络和系统安装权限后再运行

## PX4 接入前置

```bash
# 安装 PX4 源码和 Gazebo Classic 目标（会下载源码并安装系统依赖）
./scripts/install_px4_sitl.sh
```

真实 PX4 模式还需要：

1. 编译并运行 PX4 SITL Gazebo Classic；
2. 安装匹配版本的 `px4_msgs`；
3. 启动 Micro XRCE-DDS Agent；
4. 将 PX4 的位姿、IMU、雷达、光流和电量映射到 `/uav/*` 统一话题；
5. 关闭 `mock_flight`，再运行 `px4_offboard` 和任务节点。

这些步骤必须在 PX4 版本确定后完成，不能把当前 mock 模式当成比赛最终验证。

## 当前训练参数的边界

地图图片没有公开每个区域的正式中心坐标，也没有公布隔离墙正式高度；PDF 还说明现场坐标可能微调。因此 `mission.yaml` 中的坐标和 3 m 墙高都是训练参数，不是官方最终参数。正式比赛拿到现场参数后，只修改 YAML 和必要的传感器映射，不重写任务状态机。

## 一键启动比赛地图 + PX4

避免手工粘贴环境变量时和 `make` 命令粘连，可以直接执行：

```bash
cd /home/jiaxintang/桌面/bao
./scripts/run_px4_competition.sh
```

启动前会检查地图文件并清理旧的 PX4/Gazebo 进程。脚本会设置 `PX4_SITL_WORLD` 并关闭跟随无人机的相机，让世界文件里的俯视相机生效。
