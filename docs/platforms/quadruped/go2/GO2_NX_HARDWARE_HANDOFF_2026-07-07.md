> 历史归档（2026-07-07，旧狗）。文中的 `192.168.1.105`、外置 MID360、ROS 状态与排障结论不适用于当前 Go2/NX 部署；当前机器使用自带雷达。本文仅保留旧环境的来源记录，禁止按下列命令直接部署，实际操作以当前部署手册和现场复核为准。

# Go2 NX Hardware Handoff

日期：2026-07-07

这份文档专门补充 `go2 / NX` 端的硬件、传感器、底层工作区和当前阻塞，作为 `dimos + NX bridge` 迁移到 `pusai` 上继续推进时的配套 handoff。

---

## 1. 这份文档的用途

前一份 handoff 主要偏 `pusai + dimos` 软件侧。

这份文档单独回答下面这些问题：

- `go2` 端现在到底是哪台设备
- 当前真实可见的硬件构成是什么
- `MID360` 在哪里、扮演什么角色
- 当前 `NX` 上有哪些底层工作区和现成痕迹
- 当前阻塞到底是 `dimos`，还是 `MID360 / Livox / SLAM / ROS2` 这一层

---

## 2. 当前 go2 端的真实形态

当前可 SSH 的地址：

- `192.168.1.105`

当前登录方式：

```bash
ssh -J pusai unitree@192.168.1.105
```

当前登录上去后确认到的事实：

- 主机名是 `ubuntu`
- 内核是 Jetson/NX 风格：
  - `Linux ubuntu 5.10.104-tegra ... aarch64`
- Python 是：
  - `3.8.10`
- 当前系统里可见 ROS 安装：
  - `/opt/ros/foxy`
  - `/opt/ros/noetic`

这说明：

- 当前你连上的不是“纯官方 Go2 控制器开发接口”
- 而是一台挂在 Go2 体系里的 `NX` 扩展计算设备

这点非常关键，因为它决定了：

- 当前实际可控的数据链和控制链，主要都落在 `NX`
- `dimos` 官方假设的“外部主机直接对 Go2 WebRTC 直连”路线，不是当前最自然的形态

---

## 3. 当前硬件与传感器判断

### 3.1 已确认的关键外置传感器

当前已经有充分迹象表明，真实建图链的主 LiDAR 是：

- `Livox MID360`

理由：

- NX 上存在官方 Livox 工作区：
  - `/home/unitree/tmp/official_livox_ros_driver2_ws`
- NX 上有大量 `livox_ros_driver2_node_*` 日志
- 前面的排查过程中，`MID360_config.json` 的 LiDAR IP 曾经被发现写错，并做过修正

### 3.2 `MID360` 是什么位置

当前更合理的理解是：

- `MID360` 是外置 Livox 雷达
- 当前真机的建图 / 点云输入主链更依赖它，而不是完全依赖 Go2 机身自带感知

### 3.3 Go2 机身自带传感器的当前状态

目前没有拿到足够证据证明：

- Go2 机身自带 LiDAR 或机载点云重建设备，已经以一个稳定、开放的 ROS2 topic 形式接入到了当前这套 NX 工作流里

换句话说，目前不能假设：

- “不用 MID360，直接换回 Go2 自带点云就能平替”

当前更保守、也更接近事实的表述应该是：

- 当前这套能被你实际操控和排查的底层建图链，核心依赖仍然是外置 `MID360`

---

## 4. 当前 NX 上能看到的关键工作区

当前确认存在：

- 主项目：
  - `/home/unitree/projects/Embodied-Robot-Project`
- 导航/建图工作区：
  - `/home/unitree/projects/nav2go2`
- 官方 Livox driver 工作区：
  - `/home/unitree/tmp/official_livox_ros_driver2_ws`

另外还能看到一些历史或相关痕迹：

- `/home/unitree/projects/nav2go2（复件）`
- `/home/unitree/projects/nav2go2_back`
- `/home/unitree/tmp/livox_*`
- `/home/unitree/tmp/mapping_mid360*`
- `/home/unitree/.ros/log/livox_ros_driver2_node_*`

这些说明：

- 这台 NX 上确实做过较多轮 `MID360 / Livox / mapping / nav2` 方向调试
- 当前最值得信的不是文档猜测，而是“哪条 launch/driver 还能被重新拉起”

---

## 5. 当前能看到但还没跑起来的底层链

本轮检查时，NX 上没有看到活跃的这些内容：

- 没有明确活跃的 `ros2 topic list`
- 没有明确活跃的：
  - `livox`
  - `slam`
  - `lio_sam`
  - `FAST_LIO`
  - `mapping`
  - `rviz`
  - 相关 ROS2 长驻进程

这说明一个很重要的现实：

- 现在 `pusai` 看不到 `/registered_scan / odom / color_image`，并不只是跨机 DDS 的问题
- 更直接的问题是：`NX` 本地这条底层传感链本轮根本没有处于运行态

---

## 6. 当前最重要的 MID360 相关结论

这是后续接手时最该记住的几条：

### 6.1 已发现过配置错误

前面排查中已经确认：

- 某次 `MID360_config.json` 里，LiDAR IP 原来写成了：
  - `192.168.123.20`
- 后来发现真实设备更像是：
  - `192.168.123.101`

这说明：

- 底层问题里，至少已经出现过“雷达配置地址错误”这种非常硬的连接问题
- 所以后续再排 `MID360` 时，IP、网卡、端口、配置文件不能想当然

### 6.2 曾经出现过 driver 层明显改善

在改对 IP 之后，至少有过这个结论：

- `livox_ros_driver2` 最前层行为明显改善

这说明：

- `MID360` 本体并不是完全没响应
- 方向不是全错

### 6.3 但主阻塞收敛到了内存与稳定性

当前更像的主要阻塞是：

- `livox_ros_driver2` 跑起来后会快速吃内存
- 进而出现 OOM / 崩溃 / 后级节点不稳定

也就是说，现阶段最核心的问题已经不再只是：

- “驱动有没有装”

而是：

- “这条 `MID360 -> driver -> SLAM/topic` 链能不能稳定持续跑”

---

## 7. `nav2go2` 当前在这套系统里的含义

当前 NX 上的：

- `/home/unitree/projects/nav2go2`

里面能看到的 launch 包括：

- `launch/laserscan_launch.py`
- `launch/slam_launch.py`
- `launch/navigation_launch.py`
- `launch/bringup_launch.py`
- `launch/explore_launch.py`

从文件内容看，它更像：

- 在已有点云输入基础上，向 2D 激光、SLAM toolbox、Nav2 等方向组织的一套导航工作区

这意味着：

- 如果 `MID360` 的点云输入本身不稳，那么 `nav2go2` 很可能只是后级受害者
- 直接先上 `nav2go2`，不一定是最省事的排查路径

更稳妥的顺序仍然是：

1. 先把 `Livox MID360` 官方 driver 单独跑稳
2. 先验证基础点云 topic
3. 再挂 `slam / nav2go2`

---

## 8. 当前与 `dimos` 的真实关系

这份硬件 handoff 的核心判断是：

- 当前最大的物理层/底层阻塞，在 `NX + MID360 + ROS2` 这边
- 不在 `dimos` 本身

更具体说：

- `dimos` 当前已经基本完成了入口准备、蓝图注册、环境搭建
- 但如果 `NX` 上没有一个稳定输出的：
  - 点云 topic
  - odom topic
  - 可见相机 topic

那么 `dimos` 无论多完善，也吃不到真数据

所以后续在 `pusai` 上继续时，要一直警惕一个误区：

- 不要把底层传感/SLAM 问题误改成上层 `dimos` 问题

---

## 9. 当前最推荐的 go2/NX 侧下一步

### Step 1：先只做最小 `MID360` 驱动验证

先不要一上来拉整套 `nav2go2`。

优先去看：

- `/home/unitree/tmp/official_livox_ros_driver2_ws`
- `/home/unitree/tmp/official_livox_ros_driver2_ws/src/livox_ros_driver2/README.md`

目标是：

- 用官方 driver 最小命令先拉起 `MID360`
- 看它能不能稳定产出点云 topic

### Step 2：确认 driver 是否稳定，而不只是“能短暂启动”

这里要重点观察：

- 启动后是否持续输出
- 是否很快内存暴涨
- 是否被 OOM kill
- 日志里是否有网络/包格式/设备断连问题

### Step 3：只有在 Step 1/2 稳定后，再接 `nav2go2`

如果连 driver 都不稳，就不建议继续往上叠：

- `laserscan`
- `slam`
- `navigation`
- `dimos bridge`

### Step 4：一旦 NX 上出现稳定 topic，再回 `pusai`

一旦 NX 真正稳定发布 topic，就立刻回 `pusai` 做：

```bash
source /opt/ros/humble/setup.bash
ros2 topic list
```

如果 `pusai` 仍然看不到，那时才值得专门排：

- DDS
- ROS domain
- 跨机发现
- Foxy/Humble 兼容性

---

## 10. 当前建议不要做的事

- 不要假设 Go2 自带雷达当前已经能直接平替 `MID360`
- 不要在 `MID360` driver 未稳定前，继续深改 `dimos` 上层逻辑
- 不要把 `nav2go2` 的失败自动解释为 `dimos` 失败
- 不要忘记当前登录的是 NX，不是理想化的“官方 Go2 WebRTC 开发入口”

---

## 11. 最短接手说明

如果新的 Codex 在 `pusai` 上继续干活，关于 `go2/NX` 这侧可以直接这样理解：

1. `192.168.1.105` 对应的是一台 NX，系统里有 `ROS Foxy + Noetic`
2. 当前真实建图主 LiDAR 更像是外置 `Livox MID360`
3. NX 上存在：
   - `official_livox_ros_driver2_ws`
   - `nav2go2`
4. 当前核心阻塞不是“有没有仓库”，而是：
   - `MID360 -> driver -> 稳定点云/topic` 这一层还没有被重新稳定拉起
5. `dimos` 只是上层消费者，当前不是第一阻塞点
