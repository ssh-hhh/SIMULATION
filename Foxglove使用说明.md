# Foxglove 调试使用说明

> DGE 进程内建 Foxglove WebSocket 服务（无需 ROS/rosbridge），用于看图像、3D 跟踪场景与曲线。
> 更新：2026-10-13。配套：《DGE_部署Daedalus仿真指南.md》。

## 1. 连接（三步）

```bash
# 1) 起闭环（部署指南），DGE 日志出现: foxglove: WebSocket server on ws://0.0.0.0:8765
# 2) Foxglove Studio（桌面版 / app.foxglove.dev）：
#    Open connection → Foxglove WebSocket → ws://localhost:8765（远程机器用其 IP）
# 3) Import layout → $DGE/foxglove_layouts/dge_debug_layout.json
```

- DGE 只在有客户端连接时才推流；**没连 Foxglove 时什么都不发**，属正常。
- 图像限频约 15fps；全速画面看 DGE 自带 OpenCV 窗口（标题 `Standard Test`，`q` 退出）。
- 网页版通过 https 打开时浏览器会拦 `ws://`，请用桌面版。

## 2. 话题（DGE 实际注册的）

| 话题 | 类型 | 内容 |
|---|---|---|
| `/detector/result_img/compressed` | CompressedImage | 带识别框结果图（布局主视图） |
| `/camera/image` | CompressedImage | 同一份 JPEG（备用） |
| `/detector/number_img` | CompressedImage | 数字分类图案拼接图 |
| `/camera/annotations` | ImageAnnotations | 装甲板四点框 |
| `/camera_info` | CameraCalibration | 内参（随 `/tf` 重发） |
| `/tf` | FrameTransforms | `odom→gimbal_link`（云台姿态）、`gimbal_link→camera_optical_frame`（外参） |
| `/dge/scene` | SceneUpdate | 3D 实体：`detections`/`tracked_target`/`self`/`aiming_point` |
| `/dge/prediction` | SceneUpdate | 预测轨迹 + 幽灵装甲板 |
| `/dge/debug` | json | 曲线字段（见 §4） |
| `/tracker/target`、`/tracker/info` | json | 目标状态 / 抖动指标 |
| `/detector/armors` | json | 识别装甲板列表 |
| `/detector/marker`、`/tracker/marker` | cdr MarkerArray | ROS 风格 Marker |
| `/aiming_point` | cdr PointStamped | 世界系瞄准点 |

坐标系：`odom`（原点=机器人云台处，X 前 / Y 左 / Z 上）。

## 3. 布局与 3D 面板

| 面板 | 内容 |
|---|---|
| 两个 Image | 结果图 / 数字分类图 |
| 3D | `/dge/scene` + `/dge/prediction` + `/tf`（**核心面板**） |
| 五个 Plot | 抖动（`position_diff`/`yaw_diff`）、目标状态（`v_yaw/radius_1/radius_2/dz`）、指令（`yaw/pitch` vs `gimbal_yaw/gimbal_pitch`）、EKF 一致性（残差/`nis`/`nees`/`recent_nis_failures`）、性能（`fps`/`armors`/`targets`） |
| RawMessages | `/tracker/target`、`/dge/debug` |

3D 面板跟随模式（布局默认第一人称）：`camera_optical_frame`=枪手视角（推荐）、`gimbal_link`=云台跟随、
`follow-position`+`gimbal_link`=只跟位置、留空=固定全景。
场景实体含义：`detections`=识别板；`tracked_target`=EKF 的板/中心/半径圈/速度箭头；
`self`=光轴/瞄准方向；`aiming_point`=瞄准点；`target_prediction`=预测。

## 4. 曲线字段与判读

| 字段 | 正常 | 异常 |
|---|---|---|
| `/tracker/info.position_diff` (m) | 稳态 ~1e-3，小尖峰 | 持续大 → 外参/检测问题 |
| `/tracker/info.yaw_diff` (rad) | 稳态小 | 持续振荡 → 观测噪声/换板 |
| `/dge/debug.yaw` vs `gimbal_yaw` | 同向、差值为跟随滞后 | 反向=符号错；发散=外参错 |
| `/dge/debug.pitch` vs `gimbal_pitch` | 同上（抬头为负） | 同上 |
| `/tracker/target.radius_1/radius_2` | 都在 (0.05, 0.5) 内 | 掉到 0.05 附近 → 即将 `Target diverged` |
| `/tracker/target.v_yaw` | 静止≈0 | 静止目标大 → 发散 |
| `/dge/debug.nis / nees / recent_nis_failures` | 稳态小、平稳，换板/机动瞬间允许尖峰 | `recent_nis_failures` 持续 ≥0.4 → 即将判定 Bad Converge 丢目标；长期偏大 → 过程/观测噪声参数不匹配 |

`/dge/debug` 字段：`fps / yaw / pitch（指令）/ gimbal_yaw / gimbal_pitch（实测）/ shoot / armors / targets / ekf[] / aim_x/y/z / aim_yaw`，
另有 EKF 一致性曲线：`residual_yaw / residual_pitch / residual_distance / residual_angle`（新息）、`nis / nees`（归一化新息/误差平方，越小越一致）、`recent_nis_failures`（滑动窗口 NIS 超阈值比例，持续 ≥0.4 会触发"Bad Converge"丢目标）。

## 5. 按问题查：数据这么多，先看哪个？

| 你怀疑/想看什么 | 先看哪个面板或话题 | 怎么看 |
|---|---|---|
| 识别不到 / 识别慢 | `/detector/result_img`（画框图）、`/detector/number_img`、`/detector/armors` | 框有没有、框正不正、数字对不对 |
| 整套跑多快 | 性能面板：`/dge/debug.fps`（配合 `/dge/debug.armors/targets`） | 只看稳态均值；对比实验保持同一场景 |
| 跟踪抖不抖 | 抖动面板：`position_diff` / `yaw_diff` | 稳态 ~1e-3 m 量级，换板小尖峰正常 |
| 跟踪对不对（空间） | 3D 面板 `/dge/scene` | 检测板（灯色）与 EKF 板（半透明黄）应重合；转云台时世界系原地不动 |
| 半径/角速度估得准不准 | 目标状态面板 `v_yaw / radius_1 / radius_2 / dz` | v_yaw 1s 内收敛到约 ±转速；半径在 (0.05, 0.5) |
| EKF 是不是快崩了 | EKF 一致性面板 `nis / nees / recent_nis_failures` | `recent_nis_failures` 持续爬向 0.4 = 预警丢目标 |
| 瞄准/云台指令对不对 | 指令面板 `yaw` vs `gimbal_yaw`、`pitch` vs `gimbal_pitch` | 同向、差值是跟随滞后；反向=符号错、发散=外参错 |
| 开火时机 | `/dge/debug.shoot`；3D 里 `aiming_point` 与目标的贴合度 | `shoot=1` 表示当前帧建议开火 |
| 和真值差多少 | `/sim/ground_truth`（RawMessages 看原值，目标状态面板看 vyaw） | 位置差一个常量平移（原点不同）；`yaw/vyaw` 可直接对比 |
| 想看原始数字 | RawMessages：`/tracker/target`、`/dge/debug`、`/detector/armors` | 顶部下拉切话题，JSON 树里点字段 |

**自己加面板/曲线（两步）**：Foxglove 左上 `+` 或面板右上菜单 → 加 `Plot` / `RawMessages` / `3D` → 
在 Series/路径里直接填 §2/§4 的字段名（如 `/dge/debug.fps`）。改完布局记得导出保存；重新导入 `dge_debug_layout.json` 会覆盖当前布局。
**3D 面板显隐**：左侧 Topics 一栏勾选/取消 `/dge/scene`、`/dge/prediction`、`/detector/marker`、`/tracker/marker` 等；
点场景里的实体可看其字段。图像话题限频约 15fps，全速画面以 DGE 的 OpenCV 窗口为准。

## 6. 两个核心调试工作流

**A. 外参验证（30 秒）**：锁定静止目标 → 缓慢转动云台/底盘 → 看 `/dge/scene` 的 `detections`：
- ✅ 世界系原地不动；
- ❌ 随 yaw 画弧线 → 相机杆臂 `t_camera2gimbal` 错（应为 `[0.1374,0.0020,0.1526]`）；
- ❌ 方向整体偏 90°/反向 → `imu_left/right`、`yaw_sign/pitch_sign` 错。

**B. 发散排查**：日志 `Target diverged!` + `radius_1/radius_2` 乱跳时，先做 A；
A 通过仍偶发 → 属单装甲板弱观测/装甲尺寸假设差异，可容忍或后续调 tracker。目标出画/被遮挡后的
`temp_lost` 重锁属正常。开火链路看 `/dge/debug.shoot`（1=已建议开火）。

## 7. FAQ

| 问题 | 处理 |
|---|---|
| 连不上 8765 | 确认 DGE 在跑、日志有 WebSocket 行；`ss -ltnp \| grep 8765`；远程用对 IP |
| 面板全空 | 没客户端时 DGE 不推流，属正常；或面板引用了不存在的话题 |
| 3D 空白 | `fixedFrame/followTf` 用 `odom`/`gimbal_link`/`camera_optical_frame` |
| 曲线无数据 | series 的 `path` 必须是 §4 的字段（如 `/dge/debug.yaw`） |
| 布局导入报错 | 面板类型名须为 Foxglove 注册名（`ImageViewPanel`/`3D`/`Plot`/`RawMessages`） |
| 时间戳奇怪 | 消息时间戳都是 DGE 主机本地时钟，无需对时 |

布局文件本身的说明见 `$DGE/foxglove_layouts/README.md`。
