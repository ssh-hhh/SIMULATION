# DGE × Daedalus 仿真部署指南

> 把 DGE 自瞄框架接入 Daedalus 仿真器，实现「图像 → 识别 → 跟踪 → 瞄准 → 云台跟随」闭环。
> 更新：2026-10-13。配套：《Foxglove使用说明.md》（调试）、《环境配置与使用指南.md》（装环境）。
>
> **路径约定**：两个仓库可放任意位置，下文用 `$DGE`（DGE 仓库）、`$SIM`（仿真器仓库）代指：
>
> ```bash
> DGE=/path/to/DGE_vision_26                 # 例: ~/DGE_vision_26
> SIM=/path/to/bevy_robomaster_simulator-master
> ```
>
> 脚本会自动定位（脚本同级 / 系统桌面 / 环境变量 `DGE_DIR`、`SIM_DIR` 覆盖），不写死路径。

---

## 1. 快速开始

```bash
# 终端 1：仿真器（必须先启动；必须从仓库根目录启动，见 §8）
cd "$SIM" && source ~/.cargo/env && cargo run --release

# 终端 2：DGE
cd "$DGE" && ./build/standard configs/daedalus.yaml
```

仿真器窗口里按 **F5**（自瞄订阅开关，不开则云台指令被忽略）；按 ↑↓←→ 手动转云台、
WASD 开底盘、IJKL 移动敌方假人。

## 2. 系统组成与约定

```
仿真器(Rust/Bevy) ⇄ 共享内存 /tmp/talos_ipc_* ⇄ DGE(C++/OpenVINO)
   ↑ 发布 1440x1080 图像 + 云台位姿        ↑ 发回 GimbalCmd(yaw/pitch/fire)
```

- **无 ROS2**；先启动仿真器（共享内存由它创建），DGE 只 attach。
- **F5 是总开关**：仿真器只在 F5 开启后才接收 DGE 的云台指令。
- **图像是背压握手**：没人消费位姿通道时图像流会停住（正常现象，不是故障）。
- DGE 启动即内建 Foxglove 服务：`ws://localhost:8765`（用法见另一文档）。

## 3. 前置条件

| 项目 | 要求 |
|---|---|
| 系统 | Ubuntu 22.04（X11），可跑 Vulkan 的 GPU（NVIDIA 最佳） |
| Rust | ≥ 1.85（仿真器 edition 2024） |
| CMake | ≥ 3.23 |
| OpenVINO | 2024.x（DGE 推理用；**安装路径任意**，脚本自动探测并写入构建配置） |
| 其他 | OpenCV 4.x、Eigen3、yaml-cpp、nlohmann-json、fmt（apt 安装） |

装环境的完整步骤（apt 清单、镜像加速、OpenVINO/CMake/Rust 安装）见《环境配置与使用指南.md》。

## 4. 部署

DGE 维护仓库 **`github.com/RM-DGE/DGE_vision_26` 已包含仿真后端**，克隆后直接构建即可：

```bash
# 1) 取得 DGE（已含仿真后端）
git clone https://github.com/RM-DGE/DGE_vision_26.git "$DGE"

# 2) 若 OpenVINO 不在 /opt/intel/openvino_2024.6.0：
#    编辑 "$DGE/CMakeLists.txt" 里 set(OpenVINO_DIR "...") 为你的实际路径

# 3) 构建（产物 build/standard）
cd "$DGE" && mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j"$(nproc)"
```

若你用的是**自己的 DGE 版本**：需要新增 `io/Daedalus/`（7 个文件）与 `tools/Foxglove/`，
并对 5 个文件做小改动（`io/camera.cpp`、`io/cboard.hpp`、`io/cboard.cpp`、`src/standard.cpp`、`io/CMakeLists.txt`），
另把 `configs/daedalus.yaml` 拷过去；具体 diff 参考维护仓库提交 `87ce5e8`（添加 daedalus）与 `69131f1`。

改过 DGE 源码后重新构建：`cmake --build "$DGE/build" -j"$(nproc)"`。
**只改 `configs/daedalus.yaml` 不需要重新编译**，直接重启 DGE 即可。

## 5. 适配层做了什么

| 文件 | 作用 |
|---|---|
| `io/Daedalus/shm_client.hpp` | 共享内存客户端（mmap + 三缓冲；心跳必须用 UNIX_EPOCH 比较） |
| `io/Daedalus/shared.hpp` | 单例：连接 + 云台四元数队列 + 轴系/符号参数 |
| `io/Daedalus/daedalus_camera.*` | 收图转 BGR；**每帧消费全部位姿通道**并压 IMU 队列 |
| `io/Daedalus/daedalus_cboard.hpp` | `imu_at()` 插值；`send()` 换算并发送云台指令 |
| 补丁 | 给 camera/cboard/standard.cpp/CMakeLists 加 `daedalus` 工厂分支 |

三条关键行为（排错先看这里）：
1. **位姿通道必须全消费**，否则图像发布阻塞（启动时预发约 42 帧后停住）。
2. **无目标时发 `distance_m=-1` 哨兵**：仿真器移除目标、云台原地保持（不会回弹）。
3. **坐标系与符号（2026-10-13 修正）**：仿真器发布的四元数已是 DGE/ROS 轴系，
   DGE 侧 `imu_left/right` 必须是**单位四元数**；正确符号是 `yaw_sign=1`、`pitch_sign=1`。
   相机安装角 25°（`gimbal_link = 云台 × Rx(25°)`），故
   `t_camera2gimbal = (0.1374, 0.0020, 0.1526)`（由实测 POSE_CAMERA 杆臂 `(0.060,0.002,0.196)` 旋转得到）。
   这三处填错会分别表现为：**开自瞄偏向一边 / 云台反向 / 一动底盘就发散乱摆**。

## 6. 配置（`configs/daedalus.yaml`）

| 参数 | 值 | 说明 |
|---|---|---|
| `enemy_color` | `blue` | 敌方假人是蓝方；红方改 `red` |
| `camera_name` / `cboard_name` | `daedalus` | 仿真后端；**真机删掉 `cboard_name` 行** |
| `camera_matrix` | fx=fy≈1303.68, c=720/540 | 与仿真器 1440×1080 匹配（来自共享内存内参） |
| `R_camera2gimbal` | `[0,0,1,-1,0,0,0,-1,0]` | 相机光轴 z→link +x（REP-105） |
| `t_camera2gimbal` | `[0.1374,0.0020,0.1526]` | 见 §5 |
| `daedalus.yaw_sign/pitch_sign` | `1 / 1` | 见 §5 |
| `daedalus.imu_left/right` | `[1,0,0,0]` | 见 §5 |
| `daedalus.smooth_alpha` | 默认 0.5 | 指令低通；抖动大可调小 |
| tracker / aimer / shooter | 默认 | `min_detect_count=5`、切入/切出角 60°/20°、`auto_fire=true` |

## 7. 验证

启动日志（DGE `logs/` 最新文件）应含：

```
foxglove: WebSocket server on ws://0.0.0.0:8765
foxglove: camera fx=1303.68 ... extrinsics t=(0.137,0.002,0.153)   ← 回显 yaml 外参，一眼核对
[Daedalus] shm connected (producer alive: true)
```

探针（在 `$SIM` 下运行，只读、可与闭环并存）：

```bash
cargo run --release -p talos-ipc --example probe      # 图像seq/云台姿态/指令/心跳
cargo run --release -p talos-ipc --example gt_probe   # DGE 指令 vs 真值目标方位（自检方向）
cargo run --release -p talos-ipc --example probe2     # 内参+位姿（勿与闭环同跑）
cargo run --release -p talos-ipc --example calib watch         # 云台姿态（V0=当前/V1=历史）
cargo run --release -p talos-ipc --example calib -- send -135 18  # 直发指令（需 F5）
```

静止自检：`probe2` 应显示云台四元数 `w≈0.9763, y≈-0.2164`（= 抬头 25°）、相机杆臂 `(0.060,0.002,0.196)`。
方向自检：`gt_probe` 里 DGE 的 `cmd yaw ≈ 真值方位角`、`cmd pitch ≈ -真值仰角`（差几度内属正常，瞄准点是装甲板不是整车中心）。
直观验证：锁定静止目标后转动云台，Foxglove `/dge/scene` 的装甲板方块应世界系原地不动。

## 8. 常见问题

| 现象 | 处理 |
|---|---|
| DGE 报等图像超时 / `probe` 图像 seq 不动 | 先启动仿真器；确认有消费者（DGE 或 `calib watch`）在跑 |
| F5 后云台不动 | ① F5 是否开启；② 无有效目标时 DGE 发 -1 哨兵、云台保持是正常设计 |
| 开 F5 偏向一边 / 反向 / 一动就乱摆 | 对照 §5 三条约定检查（轴系、符号、杆臂） |
| 启动报 magic/version 不匹配 | `rm -f /tmp/talos_ipc_meta /tmp/talos_ipc_image_pool` 后重启仿真器 |
| 仿真器黑屏/找不到 `DART.glb` | 必须 `cargo run`（仓库根目录），别直接跑二进制 |
| 识别不到敌方 | `enemy_color` 不对 / 目标不在画面 / `min_confidence` 过高 |
| 日志偶发 `Target diverged!` | 单装甲板弱观测 + 装甲尺寸假设差（仿真 133.8×52.1 vs 假设 135×56mm）；少量可容忍 |

## 9. 备注

- 弹道：DGE 用 24.5 m/s、g=9.7833；仿真器弹速 25.0、g=9.81，远距离落点有偏差，要打准可两边对齐。
- 仿真器 Foxglove 图像限 15fps；DGE 自带 OpenCV 窗口是全速的。
- 早期 `talos26-master` 路线已移除，仅在《环境配置与使用指南.md》留作历史记录。
