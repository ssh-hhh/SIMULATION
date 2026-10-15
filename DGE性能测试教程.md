# DGE 性能测试操作教程（附 config.toml 参数手册）

> 面向第一次做测试的同学。按本文从上往下做一遍，你就能回答四个问题：
> ① 我的识别+跟踪+瞄准整套跑多快？② 谁最慢？③ EKF 稳不稳？④ 打不打得到、怎么调参数？
> 相关文档：《DGE_部署Daedalus仿真指南.md》《Foxglove使用说明.md》《环境配置与使用指南.md》。
> 更新：2026-10-14（含 `/dge/debug` EKF 一致性曲线）。

---

## 0. 一分钟名词表

| 词 | 指什么 | 在哪 |
|---|---|---|
| DGE | 你的自瞄程序 | `~/桌面/DGE_vision_26` |
| 仿真器（Daedalus） | 渲染战场、发布图像/云台姿态、接收云台指令 | `~/桌面/simulation/bevy_robomaster_simulator-master` |
| 识别器 | YOLO 检测装甲板（`yolo_name` 选模型） | `tasks/auto_aim/yolos/` |
| 跟踪器 / EKF | 由观测估计敌方整车状态（位置/速度/yaw/角速度/半径…） | `tasks/auto_aim/target.cpp` |
| 决策器 | 选瞄准点 + 判开火（Aimer / Shooter，MPC 是另一条线） | `tasks/auto_aim/aimer.cpp` |
| 闭环 | 图像→识别→估计→瞄准→云台指令→仿真器执行 | 两个程序之间走共享内存 |

---

## 1. 启动与自检（固定流程）

```bash
# 终端 1：仿真器（必须先启动；必须从仓库根目录 cargo run）
cd ~/桌面/simulation/bevy_robomaster_simulator-master
source ~/.cargo/env && cargo run --release

# 终端 2：DGE
cd ~/桌面/DGE_vision_26
export LD_LIBRARY_PATH=/opt/intel/openvino_2024.6.0/runtime/lib/intel64:$LD_LIBRARY_PATH
./build/standard configs/daedalus.yaml
```

启动顺序不能反（共享内存由仿真器创建）。若报共享内存 magic/version 错：

```bash
rm -f /tmp/talos_ipc_meta /tmp/talos_ipc_image_pool    # 然后重开仿真器
```

**DGE 自检**（`logs/` 最新日志）：应有三行关键信息

```
foxglove: WebSocket server on ws://0.0.0.0:8765
foxglove: camera fx=1303.68 ...
[Daedalus] shm connected (producer alive: true)
```

**仿真器窗口按键**（测试全靠这几颗键）：

| 按键 | 作用 | 测试用途 |
|---|---|---|
| **F5** | 自瞄订阅开关 | 不开则 DGE 云台指令被忽略 |
| W A S D | 己方底盘移动 | 制造"自己动"的场景 |
| ↑ ↓ ← → | 己方云台手动转动（仅 F5 关闭时） | 对照检查 |
| 空格 | 己方发射 | 手动打靶 |
| I J K L | **移动当前控制的假人** | 拉远/拉近/躲遮挡 |
| **U** | 假人小陀螺开关（持续转） | 加压核心按键 |
| Q | 己方小陀螺开关 | 模拟自身旋转 |
| `[` / `]` | 假人底盘滚转（左右倾斜 ±20°） | 让装甲板左右摇摆（加观测噪声） |
| `;` / `'` | 假人底盘俯仰（前后倾斜 ±20°） | 让装甲板前后摇摆 |
| F V C B | 拧假人云台（炮塔） | ⚠️ 不影响装甲板位置（板在底盘上），自瞄测试用不到 |
| **Tab** | 切换控制的假人（默认是英雄，切一下才是步兵） | 换目标（大/小装甲板） |
| Shift+C | 循环更换假人装甲板数字贴纸 | 考分类器/优先级 |
| F2 / F3 | 截图 / 切换视角 | 留证 |

**Foxglove**：连 `ws://localhost:8765` → `Layout → Import layout` →
`~/桌面/DGE_vision_26/foxglove_layouts/dge_debug_layout.json`。
注意：DGE 只在**有客户端连接时**才推流；图像话题限频约 15fps（全速画面看 DGE 的 OpenCV 窗口，标题 `Standard Test`，按 `q` 退出）。

---

## 2. 你会用到的六个"仪表盘"

| # | 指标 | 在哪看 | 正常范围 / 判读 |
|---|---|---|---|
| 1 | 端到端帧率 FPS | OpenCV 窗口左上角；Foxglove「性能」面板 `/dge/debug.fps`（同面板还有 `armors/targets`） | 越高越好；对比实验只看它 |
| 2 | CPU 占用 | `pidstat -p $(pgrep -x standard) 1`（sysstat 包）或 `htop` | 推理一般吃满 1~4 核 |
| 3 | 跟踪抖动 | Foxglove「跟踪抖动」面板：`/tracker/info.position_diff`、`yaw_diff` | 稳态 ~1e-3 m 量级，小尖峰正常；持续大 → 外参/检测问题 |
| 4 | EKF 状态与真值 | 「目标状态」面板：`/tracker/target.*` vs `/sim/ground_truth.targets[0].vyaw` | 小陀螺时 `v_yaw` 1s 内收敛到约 ±转速；`radius_1/2` 在 (0.05, 0.5) |
| 5 | EKF 一致性（新） | 「EKF 一致性」面板：`residual_* / nis / nees / recent_nis_failures` | 稳态小且平；换板瞬间尖峰正常；`recent_nis_failures` 长期 ≥0.4 → 快丢目标 |
| 6 | 命中率（直击） | **仿真器窗口左下角**小字：`auto-aim=ON total=… accurate=… pct=…` | 最终效果分；只统计**直击**：弹丸第一次碰撞就是装甲板才算（能量机关命中不算），先弹到地面/结构再碰板不算 |

> 注意：指标 1 在 DGE 里统计的是整个循环（读图+识别+跟踪+决策+显示+推流），做 A/B 对比时要保证 **Foxglove 连接状态一致、目标位姿一致**，否则数字不可比。

---

## 3. `config.toml` 参数手册（仿真器）

文件：`~/桌面/simulation/bevy_robomaster_simulator-master/config.toml`
**热重载**：运行中保存会打印 `Config reloaded successfully`。其中"每帧读取"的参数（底盘转速 `rotation_speed`、起转加速度 `yaw_acceleration`、云台 PID、pitch 限位、物理步长、弹速）立即生效；**分辨率、光照、窗口、以及场景生成时读取到车辆组件里的参数（如 `max_speed`、`linear_acceleration`）需要重启仿真器**。做正式对比实验前建议重启一次，保证干净状态。

### [vehicle] —— 敌方（和你自己）运动能力，加压的核心

| 参数 | 现值 | 含义 | 怎么用它做实验 |
|---|---|---|---|
| `rotation_speed` | 10.00 | 小陀螺最高转速（rad/s），按 **U** 后假人以此速度持续转 | **核心加压旋钮**：8→10→12→14 逐档测试 v_yaw 收敛与命中率 |
| `yaw_acceleration` | 30.0 | 起转加速度（rad/s²），决定"说转就转"的程度 | 调大=起转更突然（同等转速更苛刻）；调小=缓起转 |
| `tilt_rotation_speed` | 3.0 | 底盘俯仰/翻滚摇摆速度（按键 `[`/`]`、`;`/`'` 触发） | 调大=车更晃，观测更脏（考 EKF 抗噪） |
| `gimbal_rotation_speed` | 3.0 | 手动拧假人云台（炮塔）速度 | 云台不带装甲板，对自瞄测试影响很小 |
| `gimbal_pitch_limit` | 0.785 | 云台俯仰限位（rad，约 ±45°） | 一般不动 |
| `max_speed` | 6.0 | 底盘最大速度（m/s） | 拉高让假人平移更快（**需重启仿真器**） |
| `linear_acceleration` | 6.0 | 底盘加速度（m/s²） | 同上，配合 max_speed |
| `acceleration_exponent` | 2.5 | 非线性加速曲线的指数 | 一般不动 |

### [vehicle.gimbal_pid] —— 仿真云台对自瞄指令的跟踪能力

给你发出去的 yaw/pitch 指令做闭环。**每帧读取，热重载生效。**

| 参数 | yaw 现值 | pitch 现值 | 含义 |
|---|---|---|---|
| `kp` | 25.0 | 15.0 | 比例增益，调低=云台"迟钝"（压你的指令质量） |
| `ki` / `kd` | 0 / 0.1 | 0 / 0.1 | 积分/微分 |
| `integral_limit` | 0.5 | 0.5 | 抗饱和 |
| `max_rate` | 40.0 | 10.0 | 角速度饱和（rad/s） |

实验：把 `max_rate` 降到 10/5 → 观察 `/dge/debug.yaw` 与 `gimbal_yaw` 的滞后变大、命中率下降——这是"验证你的瞄准指令在有控制延迟时是否还成立"。

### [projectile] —— 弹丸与开火

| 参数 | 现值 | 含义 | 注意 |
|---|---|---|---|
| `speed` | 25.0 | 出膛速度 m/s | **DGE 里硬编码 24.5**（standard.cpp），要打准把它改成 24.5（或改 DGE）；重力两边也有小差（9.81 vs 9.7833） |
| `cooldown` | 0.05 | 射频间隔（20 发/s 上限） | 命中率分母 |
| `lifetime` | 5.0 | 弹丸存活 | 一般不动 |
| `diameter` / `mass` / `friction` | 0.017 / 0.0032 / 1.1 | 弹丸物理 | 一般不动 |

### [physics]

| 参数 | 现值 | 含义 |
|---|---|---|
| `fixed_hz` / `substep_count` | 120 / 8 | 物理步频与子步；调高更真实但更吃 CPU |

### [capture.color] —— 相机输出分辨率

现值 1440×1080（与 DGE `camera_matrix` fx≈1303.68 匹配）。
⚠️ **改分辨率必须同步改 `DGE/configs/daedalus.yaml` 的 `camera_matrix`**，否则测距全错。改完重启仿真器。

### [render] / [window] / [preview] / [debug] —— 性能与画质

| 参数 | 现值 | 作用 | 测试取向 |
|---|---|---|---|
| `render.illuminance` | 500.0 | 光照强度 | 调低=暗光识别压力测试（**重启生效**） |
| `render.shadows` | true | 阴影 | 关掉可省渲染性能 |
| `window.present_mode` | auto_no_vsync | 垂直同步策略 | 保持默认 |
| `preview.enabled` | true | 屏幕预览渲染 | 默认开；关掉省性能但看不到窗口 |
| `debug.diagnostics` | true | 控制台帧率诊断 | 排性能瓶颈时看它 |
| `debug.egui` / `inspector` | false / false | 调试面板 | 开了明显掉帧，别乱开 |

### [camera]（自由视角）

`fov`、`follow_offset`、`free_move_speed`、`mouse_sensitivity` —— 只影响观察体验，不影响算法测试。

### [armor_perturb] —— 装甲板布局扰动（模型失配测试，默认关闭）

只影响**蓝色假人（步兵 + 英雄）**，改完保存即热重载生效；把 `enabled` 改回 `false` 立即恢复原布局。

**全局参数（作用于全部 4 块板）**

| 参数 | 现值 | 含义 | 建议实验值 |
|---|---|---|---|
| `enabled` | false | 总开关 | 测试时 true |
| `radius_scale` | 1.0 | 4 块板水平半径缩放 | 1.3~1.5（外扩，测 EKF 的 r1/l） |
| `height_offset_m` | 0.0 | 全部板统一升降（米，+ 抬高） | 0.05 |
| `height_stagger_m` | 0.0 | 前后板对(+)/左右板对(-)的高度差（米） | 0.05~0.08（测 dz 估计） |
| `yaw_offset_deg` | 0.0 | 板子整体绕底盘轴旋转（度），位置与朝向一起转 | 45（非常规布局） |

**单板 / 对向板控制（`[armor_perturb.plate]`）**

板编号以**车头（炮口）方向**为准顺时针：**1=前、2=右、3=后、4=左**（步兵/英雄通用，启动日志会打印每块板的编号与方位角，可核对）。每块可单独设：

| 字段 | 含义 |
|---|---|
| `radial_m` | 沿该板半径方向外推(+) / 内收(-)，米 |
| `height_m` | 抬高(+) / 降低(-)，米 |
| `angle_deg` | 绕底盘轴旋转该板（位置与朝向一起转），度 |
| `size_scale` | 板子尺寸缩放（1.0 = 原始） |

```toml
[armor_perturb]
enabled = true
radius_scale = 1.2            # 全局：先整体外扩
height_stagger_m = 0.05       # 全局：前后板对抬高

[armor_perturb.plate]
1 = { height_m = 0.06 }       # 单块：前板再抬高 6cm
2 = { radial_m = 0.04 }       # 对向：右板外扩 4cm
4 = { radial_m = 0.04 }       #       左板外扩 4cm（2&4 = 左右对）
3 = { angle_deg = 30 }        # 单块：后板绕车转 30°
```

用法与注意：
- 单板设置**叠加**在全局参数之上（先全局、再单板）；
- 编号核对：看启动日志 `armor perturbation: [..] plate 1 <- '1_ARMOR_ROOT' (az -0.0°, ...)`；或只给某一块设个明显偏移（如 `height_m = 0.1`）肉眼确认；
- 扰动量就是"仿真真值"（GT 只有整车位姿，没有装甲板级真值）：拿你设定的 `radius_scale / stagger / 单板偏移` 去核对 EKF 的 `radius_1 / dz` 收敛值和 `/dge/scene` 里 EKF 板与检测板的重合度；
- 偏差别急着当 bug——DGE 的 PnP 假设板宽固定（小装甲 135mm），几何改大后测距会有系统偏置，这本身就是测试内容；
- 布局改太离谱时起步阶段更易 `Target diverged`（EKF 初始半径假设 0.2m 与实际不符），属预期现象。

### [armor_hitbox] —— 装甲板判定盒（控制"打哪里才算命中"）

把模型自带的装甲网格碰撞体替换成**可调尺寸的长方体**（只影响撞击判定，不改模型外观）；
系数按"最大边=宽、中间边=高、最小边=厚"自动对应到模型局部轴（步兵/英雄通用），保存即生效。

| 参数 | 现值 | 含义 | 建议实验值 |
|---|---|---|---|
| `width_scale` | 1.0 | 板宽缩放（1.0 = 模型原样） | 1.0 |
| `height_scale` | 1.0 | 板高缩放（模型整板高 ≈12cm，DGE 检测的灯条矩形高 ≈5.6cm） | 0.46（缩到灯条矩形）|
| `thickness_scale` | 1.0 | 板厚缩放 | 1.0 |
| `debug_draw` | false | 绿色半透明盒可视化判定盒 | 调参时先开 true |

用法：调判定盒时先开 `debug_draw`，在窗口里看到绿色盒子核对位置/大小；再配合"直击才算命中"的规则（见 §7），
让命中统计逼近"打在 DGE 瞄准的那块板上"。

---

## 4. `configs/daedalus.yaml` 参数手册（DGE 侧）

文件：`~/桌面/DGE_vision_26/configs/daedalus.yaml`
**改 yaml 不用重编译，重启 DGE 即可生效。**

### 识别/推理

| 参数 | 现值 | 含义 | 实验建议 |
|---|---|---|---|
| `yolo_name` | `yolov5` | 识别模型三选一：`yolov5` / `yolov8` / `yolo11` | **性能对比主变量**：换模型重开，记 FPS vs 命中率 |
| `device` | `CPU` | OpenVINO 设备（`CPU`/`GPU`/`AUTO`） | 想试 GPU 需先装 Intel 计算运行时（见 DGE readme §3.2 第 6 条） |
| `min_confidence` | 0.8 | 检测置信度阈值 | 调低（0.5~0.6）提高远距离召回，代价是误检变多 |
| `use_traditional` | false | 传统灯条识别 | 保持 false |
| `use_roi` + `roi` | false，[420,50,600,600] | 只在 ROI 区域内检测（可提速） | 目标会离开 ROI，测速时可以开，测召回别开 |
| `enemy_color` | `blue` | 敌方颜色（假人是蓝方） | 别改错 |

### 跟踪器（EKF 外层状态机）

| 参数 | 现值 | 含义 |
|---|---|---|
| `min_detect_count` | 5 | 连续检测多少帧才确认目标（越大越稳但锁定慢） |
| `max_temp_lost_count` | 15 | 丢失多少帧内仍尝试重锁（超出才丢） |
| `outpost_max_temp_lost_count` | 75 | 前哨站特殊值 |

EKF 内层参数（过程噪声 `v1/v2`、观测噪声 `R_dig`）不在 yaml，在源码 `tasks/auto_aim/target.cpp`，改它要重编译——这正是"EKF 调参实验"的对象。

### 决策器 / 开火

| 参数 | 现值 | 含义 |
|---|---|---|
| `yaw_offset` / `pitch_offset` | 0 / 0 | 标定偏置（仿真为 0；真机才调） |
| `comming_angle` / `leaving_angle` | 60 / 20 | 高速切入/切出角（选板迟滞） |
| `decision_speed` | 7 | 判"小陀螺"的角速度阈值（rad/s） |
| `high/low_speed_delay_time` | 0.05 / 0.05 | 预测时间补偿（弹道提前量） |
| `first/second_tolerance` | 3 / 2 | 近/远距离开火角度容差（度） |
| `judge_distance` | 2 | 距离分界（m） |
| `auto_fire` | true | 自瞄自动开火；false 则只瞄准 |

### 仿真适配（别乱动，方向不对先看这里）

`daedalus.yaw_sign/pitch_sign = 1/1`、`imu_left/right = [1,0,0,0]`、`smooth_alpha`（指令低通，抖动大会让画面"抽"）。
外参 `t_camera2gimbal=[0.1374,0.0020,0.1526]`、`R_camera2gimbal` —— 详见《DGE_部署Daedalus仿真指南.md》§5，填错会"偏向一边/反向/乱摆"。

---

## 5. 改敌方状态速查（让敌人按你要的方式出现）

> ⚠️ 先记住一条容易搞错的事实：仿真里 **4 块装甲板全都挂在"底盘"节点上**（`VEHICLE/BASE/*_ARMOR_ROOT`），
> 按 **F V C B** 转动的是假人的**云台/炮塔**，**不会改变装甲板的位置与朝向**。
> 要让装甲板动起来或变朝向，用 **U**（小陀螺）或 **`[` `]` `;` `'`**（底盘摇摆）。

### 5.1 装甲板（形状 / 布局 / 图案）

| 想改什么 | 怎么改 | 生效方式 |
|---|---|---|
| 大板 / 小板 | **Tab** 选择控制的假人（英雄=大装甲板、步兵=小装甲板），再用 IJKL 摆位 | 即时 |
| 数字贴纸 | **Shift+C** 循环：哨兵→1→2→3→4→前哨→基地→5（两个假人会一起换） | 即时 |
| 半径外扩 / 高度错位 / 整体转向（全部板） | `config.toml` `[armor_perturb]` 的 `radius_scale / height_stagger_m / yaw_offset_deg` | 热重载 |
| **单板 / 对向板**偏移 | `[armor_perturb.plate]`：1=前、2=右、3=后、4=左，可分别设 `radial_m / height_m / angle_deg / size_scale` | 热重载 |
| **判定盒**大小 / 可视化 | `config.toml` `[armor_hitbox]`：`width_scale / height_scale / thickness_scale` 缩放判定盒，`debug_draw` 显示绿色盒子 | 热重载 |
| 板子的物理尺寸本身 | 改 `assets/vehicle.glb` / `HERO.glb`（Blender），配置里改不了 | 换模型文件 |

### 5.2 运动状态（转动 / 平移 / 摇摆）

| 想改什么 | 怎么改 | 生效方式 |
|---|---|---|
| 原地小陀螺 | **U** 开/关；转速 `[vehicle] rotation_speed`、起转快慢 `yaw_acceleration`（目前只有一个转向） | U 即时；参数热重载 |
| 平移速度 / 加速度上限 | `[vehicle] max_speed / linear_acceleration` | **需重启仿真器** |
| 手动平移（开远/开近/躲掩体） | Tab 选中假人 → **I J K L** | 即时 |
| 底盘摇摆（±20°） | Tab 选中假人 → **`[` / `]`**（左右滚转）、**`;` / `'`**（前后俯仰），松键保持；速度 `tilt_rotation_speed` | 按键即时；速度热重载 |
| 云台朝向 | **F/V**（俯仰）、**C/B**（偏航）——只动炮塔，基本不影响自瞄目标 | 即时 |
| 摆到固定位置（做对比实验） | 没有自动轨迹模式；用 IJKL 把假人开到参照物旁（如某条场地线），每次实验都摆回同一位置 | 即时 |

### 5.3 环境与难度

| 想改什么 | 怎么改 | 生效方式 |
|---|---|---|
| 暗光识别 | `[render] illuminance`（调低） | 需重启仿真器 |
| 分辨率 / 算力压力 | `[capture.color]`（⚠️ 改完要同步改 DGE `camera_matrix`） | 需重启仿真器 |
| 弹道一致性 | `[projectile] speed = 24.5` 与 DGE 对齐 | 热重载（下一发生效） |
| 仿真云台"迟钝度"（考验你的指令） | `[vehicle.gimbal_pid]` 的 `kp / max_rate` 调低 | 热重载 |
| 目标身份 | 仿真里红方=你自己、蓝方=假人；DGE 的 `enemy_color` 决定追谁 | 重启 DGE |

> 建议每次实验前把"敌人配置"写成一行固定描述再开测，例如：
> `小板 / 数字3 / 半径×1.3 / 10rad/s 小陀螺 / 3m / 无遮挡` —— 一次只换一个词。

---

## 6. 测试流程（每个都给了操作和记录表）

> 原则：**一次只改一个变量**；改前把基准数据记一遍；每次测试固定目标位置（比如都用 IJKL 把假人拉到你面前 3m 左右不动）。

### 测试 1：基线（5 分钟）

1. 启动两程序，F5 开自瞄，把假人（Tab 切到步兵）开进画面；
2. 静止不动，观察 30 秒：
   - FPS 数值：____
   - `/tracker/info.position_diff` 稳态量级：____
   - `/dge/debug.nis/nees` 是否平稳：____
3. 记录到表：

| 项目 | 值 |
|---|---|
| FPS（静止目标） | |
| position_diff 稳态 | |
| 能否稳定锁定（5 帧内 detecting→tracking） | |
| 静止目标 HUD 命中率（空格或自动打 1 分钟） | |

### 测试 2：识别模型对比（核心性能测试）

对 `configs/daedalus.yaml` 依次改 `yolo_name` 为 `yolov5` / `yolov8` / `yolo11`（每次重启 DGE），同一场景各跑 1 分钟：

| 模型 | FPS | 识别距离（多远还能锁） | 静止命中率 | 小陀螺命中率 |
|---|---|---|---|---|
| yolov5 | | | | |
| yolov8 | | | | |
| yolo11 | | | | |

判读：FPS 差一倍以内优先选命中率高的；如果 FPS 掉到 30 以下，看看 CPU 是否被占满（指标 2）。

### 测试 3：小陀螺转速阶梯（EKF + 决策核心测试）

改 `config.toml` 的 `[vehicle] rotation_speed`：8 → 10 → 12 → 14（可热重载），每档：

1. 按 **U** 开假人小陀螺；
2. 看「目标状态」面板：`v_yaw` 从 0 收敛到 ±转速 用了多久：____
3. 看「EKF 一致性」：`recent_nis_failures` 是否持续爬升：____
4. 看 HUD 命中率（打满 1 分钟）：____
5. 看 DGE 日志有没有 `Target diverged!` / `Bad Converge Found!`：____

| rotation_speed | v_yaw 收敛时间 | nis 均值 | diverged 次数 | 命中率 |
|---|---|---|---|---|
| 8 | | | | |
| 10（默认） | | | | |
| 12 | | | | |
| 14 | | | | |

> 这是最容易看出算法边界的测试：某个转速以上命中率骤降，就说明 EKF/决策/控制链条在那里断了。

### 测试 4：距离与遮挡（鲁棒性）

- IJKL 把假人逐步拉远（2m→4m→6m→8m），每档记录：能否锁定、识别框是否抖、命中率；
- 把假人开到前哨站/基地柱子后面（遮挡 1~2 秒再出来），看能否重锁（`max_temp_lost_count=15` 帧内）、重锁需要几帧。

| 距离 | 能否锁定 | 重锁时间 | 命中率 |
|---|---|---|---|
| 2m | | | |
| 4m | | | |
| 8m | | | |

### 测试 5：EKF 专项（用新加的面板）

1. 静止目标：`nis/nees/residual_*` 应平稳 → 记录均值；
2. U 开小陀螺：观察换板瞬间残差尖峰幅度和恢复时间；
3. 让装甲板动起来：按 **U** 开/关假人小陀螺（板随底盘转），或按住 **`[`/`]`、`;`/`'`** 让底盘左右/前后摇摆（±20°，松键保持）——看 `residual_yaw`/`residual_pitch` 是否变大后回落；
4. 对照真值：把 `/sim/ground_truth.targets[0].vyaw` 与 `/tracker/target.v_yaw` 画在一起（布局里已配好），看滞后和超调。
   ⚠️ `targets[0]` 是仿真器返回的第 1 个假人，场内有两个假人（步兵/英雄、Tab 控制）时它不一定就是你在打的那个——以 RawMessages 面板里 `/sim/ground_truth` 的实际内容为准（按 `yaw/vyaw` 形状对比即可）。
5. 想量化调 EKF：改 `tasks/auto_aim/target.cpp` 里 `v1/v2`（过程噪声）或 `R_dig`（观测噪声）→ 重编译 → 重跑本测试，对比"残差均值 vs 收敛速度"。

### 测试 6：命中率与击杀时间（最终效果）

- HUD 的 `accurate/pct` 是命中率；击杀时间用秒表（从开火到目标"死"）；
- 变量：转速、模型、距离、`auto_fire`、容差参数。

| 场景（转速/距离） | 命中率 | 击杀时间 | 备注 |
|---|---|---|---|
| 10 rad/s, 3m | | | |
| 14 rad/s, 3m | | | |

### 测试 7（可选）：录像 + 离线分段耗时

如果想知道"识别/跟踪/瞄准各占几毫秒"，需要给 DGE 的 `standard.cpp` 接两行录像（`tools::Recorder`，输出到 `records/*.avi + .txt`），再用同济的 `tests/auto_aim_test.cpp` 离线回放、逐帧打印各模块耗时。

### 测试 8：装甲板布局扰动（模型失配测试，仿真器已内置）

在 `config.toml` 的 `[armor_perturb]` 里改参数（保存即热重载，只影响蓝色假人），逐项做：

| 扰动 | 配置 | 看什么 |
|---|---|---|
| 半径外扩 30%（全板） | `enabled=true, radius_scale=1.3` | EKF 的 `radius_1` 是否收敛到新半径；`position_diff` 是否变大；命中率 |
| 高度错位 6cm（前后对） | `height_stagger_m=0.06` | `dz` 估计值（约 ±3cm 波动）与 `radius_2`；3D 里上下板是否跟上 |
| 整体旋转 45°（全板） | `yaw_offset_deg=45` | 选板/换板是否正常；`residual_yaw` 尖峰 |
| **单板**抬高 6cm | `[armor_perturb.plate]` `1 = { height_m = 0.06 }` | EKF 能否仅靠单板观测修正 dz；选板是否正常 |
| **对向两块**外扩 4cm | `2 = { radial_m = 0.04 }`、`4 = { radial_m = 0.04 }` | r1/半径估计是否分裂；换板是否正常 |
| **单板**转 30° | `3 = { angle_deg = 30 }` | 匹配/选板、`residual_yaw` 的变化 |
| **单块**缩小到 0.8 | `1 = { size_scale = 0.8 }` | PnP 测距偏置、检测鲁棒性 |
| 恢复 | `enabled=false` | 布局立即回到原样 |

记录表：

| 扰动 | radius_1 收敛值 | dz 估计 | diverged 次数 | 命中率 |
|---|---|---|---|---|
| 无（基线） | ~0.20 | ~0.014 | | |
| 半径×1.3（全板） | | | | |
| 高度错位 6cm（对） | | | | |
| 单板抬高 6cm | | | | |
| 对向外扩 4cm | | | | |

> 提醒：仿真真值没有装甲板级数据，改完布局后 EKF 的 `r1/dz` 对不对要拿你设定的扰动值核对；
> 布局改动会引入 PnP 测距系统偏置（DGE 假设板宽固定 135mm），看到远距离落点偏移先想到这一点。

---

## 7. 常见问题

| 现象 | 处理 |
|---|---|
| DGE 报等图像超时 / 图像 seq 不动 | 先启动仿真器；确认有消费者（DGE 或 `calib watch`）在跑；清共享内存重开 |
| F5 后云台不动 | 无有效目标时 DGE 发 -1 哨兵、云台保持是**正常设计**；先把假人开进画面 |
| Foxglove 面板全空 | 没客户端时 DGE 不推流；或面板引用了不存在的话题（改过布局要重新导入） |
| 曲线断断续续 | DGE 只在有目标时更新部分字段；图像限频 15fps 属正常 |
| 远距离打不准 | 先把 `[projectile] speed` 改成 24.5 与 DGE 对齐 |
| 偶发 `Target diverged!` | 单装甲板弱观测 + 仿真装甲尺寸（133.8×52.1）与 DGE 假设（135×56）差异，少量可容忍；连续出现按《部署指南》§5 查外参/轴系 |
| 机器人不动时 EKF 位置和 `/sim/ground_truth` 对不上 | 正常：DGE 原点在云台上，GT 在仿真器世界系，相差一个常量平移；对比看趋势/峰值即可，`v_yaw` 可直接对比 |
| 打中装甲板却**不计**命中？ | 命中按**直击**统计：弹丸的**第一次碰撞**就是装甲板才算；先弹到地面/中央结构、反弹后再打到板不算。 |
| 会从车体内部打到装甲板**背面**吗？ | 不会。车体里加了"挡弹体"（`BODY_SHIELD`，位于装甲环内侧、只与对方弹丸碰撞）：穿进车体的弹丸会被它挡下且**不计**命中；从外侧正常打装甲板不受影响。 |
| 偶尔打中装甲板却没计数？ | 弹丸速度快、判定盒薄时会"穿隧"（一帧跨过盒子，正面也偶尔漏检）。把 `[armor_hitbox]` 的 `thickness_scale` 调大一些（如 2.0）让判定盒更厚，命中判定会明显更稳。 |
| 打中能量机关会算命中吗？ | **不会**。命中统计只认"直击装甲板"，能量机关命中不计入统计（也没有任何日志输出）。 |
| 想核对"到底打中了哪块板" | 命中日志已移除（之前会刷屏）。需要排查时，在 `src/robomaster/armor/collision.rs` 的 `stats.increase_accurate()` 上临时加一行日志即可，验完删掉。 |
| 打两台车中间也会命中？ | 直击规则已生效：先撞到环境/结构、反弹后再打到装甲板不算。注意**出生点就在中央机关旁**，弹丸容易先撞上中央结构——测试前先用 WASD/IJKL 把车开到空地再打。 |
| 想把命中区改成灯条矩形 | `[armor_hitbox] height_scale = 0.46` + `debug_draw = true` 核对。 |
| 想杀进程 | 用 `pkill -x` 或 `pkill -f "x[x]x"`，`pkill -f "xxx"` 会连当前 shell 一起杀 |

---

## 8. 做完基础测试之后的下一步（按需）

1. **EKF 调参**：`v1/v2`、`R_dig` 对照实验（测试 5）；
2. **MPC 规划器**：把同济仓库的 `tests/planner_test_offline.cpp` 拷进 DGE 的 `tests/`，解除 `CMakeLists.txt` 里对应目标的注释即可离线运行（曲线走 UDP 9870）；需要 Daedalus 版闭环 MPC 入口时再另行添加；
3. **装甲板布局扰动**（已内置）：改 `config.toml` 的 `[armor_perturb]`（见 §3 参数手册与测试 8）——全局（半径/高度/整体转向）+ **单板/对向板**（1前 2右 3后 4左，分别设径向/高度/转角/缩放），只影响蓝色假人，热重载即时生效；
4. **离线回放**：录像 + 回放测试程序，测分段耗时、复现问题。

---

## 附：一页速查命令

```bash
# 仿真器
cd ~/桌面/simulation/bevy_robomaster_simulator-master && source ~/.cargo/env && cargo run --release

# DGE
cd ~/桌面/DGE_vision_26
export LD_LIBRARY_PATH=/opt/intel/openvino_2024.6.0/runtime/lib/intel64:$LD_LIBRARY_PATH
./build/standard configs/daedalus.yaml

# 改过 DGE 源码后重编译
cmake --build build -j24

# 共享内存残留清理
rm -f /tmp/talos_ipc_meta /tmp/talos_ipc_image_pool

# CPU 占用
pidstat -p $(pgrep -x standard) 1
```
