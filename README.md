# simulation — Daedalus 仿真器 × DGE 自瞄闭环工作区

仿真器渲染 RoboMaster 战场，通过共享内存发布 1440×1080 图像与云台位姿；DGE 做识别/跟踪/瞄准，
把云台指令发回仿真器闭环（F5 开关）。**无 ROS2**。

```bash
# 两个仓库可放任意位置，先设好变量
DGE=/path/to/DGE_vision_26                   # 例: ~/DGE_vision_26
SIM=/path/to/bevy_robomaster_simulator-master

# 终端 1：仿真器（必须先启动）
cd "$SIM" && source ~/.cargo/env && cargo run --release
# 终端 2：DGE
cd "$DGE" && ./build/standard configs/daedalus.yaml
# 仿真器窗口按 F5 开启自瞄；IJKL 把敌方假人开进视野
```

首次部署见《DGE_部署Daedalus仿真指南.md》§4（维护仓库已含仿真后端，克隆即用）。

## 文档

| 文档 | 内容 |
|---|---|
| **DGE_部署Daedalus仿真指南.md** | 部署、适配层原理、配置、验证自检、常见问题 |
| **Foxglove使用说明.md** | 连接、话题、3D 场景/曲线判读、调试工作流、FAQ |
| **环境配置与使用指南.md** | 依赖安装（apt/Rust/CMake/OpenVINO）、踩坑、按键 |
| `DGE/foxglove_layouts/README.md` | 布局文件与话题对照 |

## 三条关键约定（排错先看）

1. **先仿真器、后 DGE**（共享内存由仿真器创建）。
2. **F5 是总开关**（不开则 DGE 的云台指令被忽略）。
3. **图像是背压握手**：没人消费位姿通道时图像 seq 停住，不是故障。
4. 外参/轴系：仿真器发布的四元数已是 DGE 轴系，`imu_left/right` 用单位四元数；
   `t_camera2gimbal=(0.1374,0.0020,0.1526)`；`yaw_sign=pitch_sign=1`（详见部署指南 §5）。

## 探针速查

```bash
cd "$SIM"
cargo run --release -p talos-ipc --example probe      # 图像seq/姿态/指令/心跳
cargo run --release -p talos-ipc --example gt_probe   # 真值方位 vs DGE 指令
cargo run --release -p talos-ipc --example probe2     # 内参+位姿（勿与闭环同跑）
cargo run --release -p talos-ipc --example calib watch
cargo run --release -p talos-ipc --example frame_dump # 导图到 /tmp/daedalus_frame.ppm

rm -f /tmp/talos_ipc_meta /tmp/talos_ipc_image_pool   # 共享内存残留时清理
cd "$DGE/build" && make -j"$(nproc)"                  # 改过源码后重编译（改 yaml 不用）
```
