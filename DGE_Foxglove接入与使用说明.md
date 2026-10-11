# 向 DGE_vision_26 接入 Foxglove —— 接入与使用说明

> 目标：让 **DGE_vision_26**（潜云战队自瞄框架，sp_vision_25 衍生，无 ROS）在跑 Daedalus 仿真闭环时，把**图像、装甲板识别框、调试曲线**推到 [Foxglove Studio](https://foxglove.dev)，用于实时观察与调试。
>
> 实现方式：基于 **官方 Foxglove C++ SDK（v0.23.0）**，不依赖 ROS，不改动任何识别/估计/瞄准算法，纯增量。
>
> 本文档**自包含**：所有步骤都在这里，不依赖任何其它脚本。
>
> 最后更新：2026-10-11 ｜ 已验证：编译通过，服务在 `ws://0.0.0.0:8765` 正常，且在真实运行的仿真器上处理帧。

---

## 1. 原理与方案选型

### 1.1 Foxglove 是什么

Foxglove 是一个机器人数据可视化工具（桌面版 / 网页版 studio.foxglove.dev）。它以 **WebSocket** 或 **MCAP 文件** 为数据源，用不同面板展示图像、2D 标注、曲线、3D 场景等。

DGE 本身没有 Foxglove：真机调试靠 `tools/plotter.hpp`（发 JSON 给 PlotJuggler）+ `tools/recorder.hpp`（录像回放）。本方案给它加一个 Foxglove 数据出口。

### 1.2 为什么用官方 C++ SDK（方案对比）

| 方案 | 结论 |
|---|---|
| **手写 Foxglove WebSocket 协议** | 可行但功能弱（只有 JSON、无类型化 schema、无 MCAP），需自己维护协议实现。**不采用。** |
| **照搬某个成熟项目（如 Talos）的可视化架构** | 其 `scene_builder` / 按层注册 System 深度绑定该项目自己的 C++23 调度器，DGE 无此结构，无法照搬。**不采用。** |
| **官方 Foxglove C++ SDK** | **采用。** 官方协议实现、类型化 schema、可选 MCAP 录制。 |

### 1.3 关键点：SDK 兼容 C++17

官方 SDK 自带 **`tl::expected`**（tartanllama 的 C++17 回填实现），因此**无需把 DGE 升到 C++23、无需换编译器**——`g++-11 -std=c++17` 即可编译运行（已实测）。

### 1.4 数据流

```
DGE main 循环 (src/standard.cpp)
   camera.read → 识别 → 估计 → 瞄准 → 云台指令
        │
        ├── publish_image(img)        ──▶ /camera/image       (foxglove.CompressedImage, JPEG)
        ├── publish_annotations(...)  ──▶ /camera/annotations (foxglove.ImageAnnotations, 2D 框)
        └── publish_debug(json)       ──▶ /dge/debug          (json, 曲线)
                                             │
                                    Foxglove C++ SDK (WebSocketServer :8765)
                                             │
                                    Foxglove Studio (ws://localhost:8765)
```

### 1.5 三个通道

| 通道 | Schema | 内容 |
|---|---|---|
| `/camera/image` | `foxglove.CompressedImage` | 原始分辨率(1440×1080)带识别框的 JPEG，约 15fps（内部限频） |
| `/camera/annotations` | `foxglove.ImageAnnotations` | 装甲板四点框（绿=其它/红=红方/蓝=蓝方），LINE_LOOP |
| `/dge/debug` | JSON（RawChannel） | `fps`、`yaw`、`pitch`、`shoot`、`armors`、`targets`、`ekf[]`、`aim_*` |

---

## 2. 涉及的文件

在本项目 `DGE_vision_26/` 内新增/修改：

| 文件 | 作用 |
|---|---|
| `3dparty/foxglove/` | **新增**：vendored 官方 SDK（`include/` + `lib/libfoxglove.a` + `src/`） |
| `tools/foxglove.hpp` | **新增**：桥接层对外接口（pimpl，使用方不含 SDK 头） |
| `tools/foxglove.cpp` | **新增**：桥接层实现（创建 server/通道，发布消息） |
| `CMakeLists.txt` | **修改**：新增 `foxglove_cpp`、`dge_foxglove` 目标，`standard` 链接之 |
| `src/standard.cpp` | **修改**：创建 `FoxgloveServer` 并在主循环发布 |

---

## 3. 接入步骤（自包含）

以下步骤假设 DGE 根目录为 `$DGE`（例如 `~/桌面/DGE_vision_26`）。
前提：DGE_vision_26 已能通过**共享内存**与 Daedalus 仿真器跑通闭环（相机从共享内存取图、云台指令写回共享内存），即它能以 `./build/standard configs/daedalus.yaml` 正常跟踪。本节只增加 Foxglove 可视化，不涉及共享内存部分。

### 3.1 获取并放置官方 SDK

```bash
DGE=~/桌面/DGE_vision_26
mkdir -p "$DGE/3dparty"
wget -O /tmp/foxglove-v0.23.0-cpp.zip \
  "https://github.com/foxglove/foxglove-sdk/releases/download/sdk%2Fv0.23.0/foxglove-v0.23.0-cpp-x86_64-unknown-linux-gnu.zip"
unzip -oq /tmp/foxglove-v0.23.0-cpp.zip -d "$DGE/3dparty/"
# 结果：$DGE/3dparty/foxglove/{include,lib,src}
ls "$DGE/3dparty/foxglove"     # 应看到 include  lib  src
```

> - 若无外网，只要是同一版本（v0.23.0，`x86_64-unknown-linux-gnu`）的 SDK 包，解压到 `$DGE/3dparty/` 得到 `foxglove/` 目录即可。
> - SDK 内含头文件、`lib/libfoxglove.a`（静态库）与 `src/`（C++ 包装层源文件，需被编译）。

### 3.2 新增 `tools/foxglove.hpp`

```cpp
#ifndef TOOLS__FOXGLOVE_HPP
#define TOOLS__FOXGLOVE_HPP

#include <list>
#include <memory>
#include <string>

#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "tasks/auto_aim/armor.hpp"

namespace tools
{

// DGE 的 Foxglove 可视化桥（基于官方 Foxglove C++ SDK，非手写协议）。
// 连接：Foxglove Studio -> Open connection -> Foxglove WebSocket -> ws://<本机IP>:8765
// 通道：
//   /camera/image       foxglove.CompressedImage   (JPEG 图像)
//   /camera/annotations foxglove.ImageAnnotations  (装甲板识别框)
//   /dge/debug          json                        (曲线)
class FoxgloveServer
{
public:
  explicit FoxgloveServer(int port = 8765, std::string name = "DGE_vision_26");
  ~FoxgloveServer();

  FoxgloveServer(const FoxgloveServer &) = delete;
  FoxgloveServer & operator=(const FoxgloveServer &) = delete;

  bool has_clients() const;
  void publish_image(const cv::Mat & bgr, int quality = 70);
  void publish_annotations(const std::list<auto_aim::Armor> & armors);
  void publish_debug(const nlohmann::json & data);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace tools

#endif  // TOOLS__FOXGLOVE_HPP
```

### 3.3 新增 `tools/foxglove.cpp`

```cpp
#include "tools/foxglove.hpp"

#include <chrono>
#include <cstring>
#include <optional>
#include <utility>
#include <vector>

#include <foxglove/channel.hpp>
#include <foxglove/context.hpp>
#include <foxglove/messages.hpp>
#include <foxglove/websocket.hpp>

#include "tools/logger.hpp"

namespace tools
{
namespace
{
uint64_t now_ns()
{
  return static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch())
      .count());
}

foxglove::messages::Timestamp to_timestamp(uint64_t ns)
{
  foxglove::messages::Timestamp t;
  t.sec = static_cast<uint32_t>(ns / 1000000000ULL);
  t.nsec = static_cast<uint32_t>(ns % 1000000000ULL);
  return t;
}
}  // namespace

struct FoxgloveServer::Impl
{
  foxglove::Context context;
  std::optional<foxglove::WebSocketServer> server;
  std::optional<foxglove::messages::CompressedImageChannel> image_ch;
  std::optional<foxglove::messages::ImageAnnotationsChannel> annotations_ch;
  std::optional<foxglove::RawChannel> debug_ch;
  uint64_t last_image_ns = 0;
};

FoxgloveServer::FoxgloveServer(int port, std::string name) : impl_(std::make_unique<Impl>())
{
  impl_->context = foxglove::Context::create();

  foxglove::WebSocketServerOptions opts;
  opts.context = impl_->context;
  opts.name = std::move(name);
  opts.host = "0.0.0.0";
  opts.port = static_cast<uint16_t>(port);

  auto server = foxglove::WebSocketServer::create(std::move(opts));
  if (!server) {
    logger()->error("foxglove: WebSocketServer: {}", foxglove::strerror(server.error()));
    return;
  }
  impl_->server = std::move(*server);

  if (auto ch = foxglove::messages::CompressedImageChannel::create("/camera/image", impl_->context);
      ch) {
    impl_->image_ch = std::move(*ch);
  } else {
    logger()->error("foxglove: create /camera/image: {}", foxglove::strerror(ch.error()));
  }

  if (auto ch = foxglove::messages::ImageAnnotationsChannel::create(
        "/camera/annotations", impl_->context);
      ch) {
    impl_->annotations_ch = std::move(*ch);
  } else {
    logger()->error("foxglove: create /camera/annotations: {}", foxglove::strerror(ch.error()));
  }

  if (auto ch = foxglove::RawChannel::create("/dge/debug", "json", std::nullopt, impl_->context);
      ch) {
    impl_->debug_ch = std::move(*ch);
  } else {
    logger()->error("foxglove: create /dge/debug: {}", foxglove::strerror(ch.error()));
  }

  logger()->info("foxglove: WebSocket server on ws://0.0.0.0:{}", port);
}

FoxgloveServer::~FoxgloveServer()
{
  if (impl_ && impl_->server) impl_->server->stop();
}

bool FoxgloveServer::has_clients() const
{
  return impl_ && impl_->server && impl_->server->clientCount() > 0;
}

void FoxgloveServer::publish_image(const cv::Mat & bgr, int quality)
{
  if (bgr.empty() || !impl_->image_ch) return;
  if (!has_clients()) return;

  const uint64_t ns = now_ns();
  if (ns - impl_->last_image_ns < 66000000ULL) return;  // 限频约 15fps
  impl_->last_image_ns = ns;

  std::vector<uint8_t> buf;
  std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, quality};
  cv::imencode(".jpg", bgr, buf, params);

  foxglove::messages::CompressedImage msg;
  msg.timestamp = to_timestamp(ns);
  msg.frame_id = "camera";
  msg.format = "jpeg";
  msg.data.resize(buf.size());
  std::memcpy(msg.data.data(), buf.data(), buf.size());
  impl_->image_ch->log(msg, ns);
}

void FoxgloveServer::publish_annotations(const std::list<auto_aim::Armor> & armors)
{
  if (!impl_->annotations_ch || !has_clients()) return;

  const uint64_t ns = now_ns();
  foxglove::messages::ImageAnnotations ann;
  ann.timestamp = to_timestamp(ns);

  for (const auto & a : armors) {
    if (a.points.size() < 4) continue;

    foxglove::messages::PointsAnnotation pa;
    pa.timestamp = to_timestamp(ns);
    pa.type = foxglove::messages::PointsAnnotation::PointsAnnotationType::LINE_LOOP;
    pa.points.reserve(4);
    for (int i = 0; i < 4; ++i)
      pa.points.push_back(
        foxglove::messages::Point2{static_cast<double>(a.points[i].x),
                                   static_cast<double>(a.points[i].y)});

    foxglove::messages::Color color;
    if (a.color == auto_aim::blue) {
      color = {0.0, 0.4, 1.0, 1.0};
    } else if (a.color == auto_aim::red) {
      color = {1.0, 0.0, 0.0, 1.0};
    } else {
      color = {0.0, 1.0, 0.0, 1.0};
    }
    pa.outline_color = color;
    pa.fill_color = foxglove::messages::Color{color.r, color.g, color.b, 0.0};
    pa.thickness = 2.0;
    ann.points.push_back(std::move(pa));
  }

  impl_->annotations_ch->log(ann, ns);
}

void FoxgloveServer::publish_debug(const nlohmann::json & data)
{
  if (!impl_->debug_ch || !has_clients()) return;

  const std::string s = data.dump();
  impl_->debug_ch->log(reinterpret_cast<const std::byte *>(s.data()), s.size(), now_ns());
}

}  // namespace tools
```

### 3.4 修改顶层 `CMakeLists.txt`

在 `include_directories(${PROJECT_SOURCE_DIR})` 之后加入 Foxglove 库定义：

```cmake
##################Foxglove C++ SDK（vendored, C++17 兼容）##################
set(FOXGLOVE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3dparty/foxglove")
set(FOXGLOVE_SOURCES
    "${FOXGLOVE_DIR}/src/channel.cpp"
    "${FOXGLOVE_DIR}/src/connection_graph.cpp"
    "${FOXGLOVE_DIR}/src/context.cpp"
    "${FOXGLOVE_DIR}/src/error.cpp"
    "${FOXGLOVE_DIR}/src/fetch_asset.cpp"
    "${FOXGLOVE_DIR}/src/foxglove.cpp"
    "${FOXGLOVE_DIR}/src/mcap.cpp"
    "${FOXGLOVE_DIR}/src/messages.cpp"
    "${FOXGLOVE_DIR}/src/parameter.cpp"
    "${FOXGLOVE_DIR}/src/service.cpp"
    "${FOXGLOVE_DIR}/src/websocket.cpp"
    "${FOXGLOVE_DIR}/src/system_info.cpp"
)
add_library(foxglove_cpp STATIC ${FOXGLOVE_SOURCES})
target_include_directories(foxglove_cpp SYSTEM PUBLIC "${FOXGLOVE_DIR}/include")
target_link_libraries(foxglove_cpp PUBLIC
    "${FOXGLOVE_DIR}/lib/libfoxglove.a"
    pthread dl m
)

# DGE 的 Foxglove 桥接层（tools/foxglove.cpp）
add_library(dge_foxglove STATIC tools/foxglove.cpp)
target_include_directories(dge_foxglove PUBLIC "${PROJECT_SOURCE_DIR}")
target_link_libraries(dge_foxglove PUBLIC foxglove_cpp ${OpenCV_LIBS})
```

并把 `standard` 的链接库末尾加上 `dge_foxglove`：

```cmake
target_link_libraries(standard ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io dge_foxglove)
```

> **两点注意**：
> 1. **不要**把 `foxglove.cpp` 加进 `tools/CMakeLists.txt`（那样会与 `dge_foxglove` 目标重复编译、链接时报重复符号）。
> 2. SDK 只编译上面列出的 **12 个源文件**，**不要**用 `src/*.cpp` 通配（`remote_access.cpp` 会引入未实现的 gateway 符号，导致链接失败）。

### 3.5 修改 `src/standard.cpp`

三处改动：

```cpp
// (1) 顶部 include
#include "tools/plotter.hpp"
#include "tools/foxglove.hpp"     // ← 新增

// (2) 创建服务器（放在 tools::Plotter plotter; 之后）
  tools::Plotter plotter;
  tools::FoxgloveServer foxglove(8765);   // ← 新增

// (3) 主循环里：画完识别框后发布图像与标注（注意必须在 resize 之前）
    for(const auto& armor : armors) {
        for(int i=0; i<4; i++)
            cv::line(img, armor.points[i], armor.points[(i+1)%4], {0,255,0}, 2);
    }

    // Foxglove：推送（原始分辨率）图像 + 装甲板识别框
    foxglove.publish_image(img);
    foxglove.publish_annotations(armors);

    // （在绘制文字之后）发布调试曲线
    {
      nlohmann::json dbg;
      dbg["fps"] = fps;
      dbg["yaw"] = command.yaw;
      dbg["pitch"] = command.pitch;
      dbg["shoot"] = command.shoot ? 1 : 0;
      dbg["armors"] = static_cast<int>(armors.size());
      dbg["targets"] = static_cast<int>(targets.size());
      if (!targets.empty()) {
        auto x = targets.front().ekf_x();
        dbg["ekf"] = std::vector<double>(x.data(), x.data() + x.size());
      }
      if (aimer.debug_aim_point.valid) {
        auto a = aimer.debug_aim_point.xyza;
        dbg["aim_x"] = a[0]; dbg["aim_y"] = a[1];
        dbg["aim_z"] = a[2]; dbg["aim_yaw"] = a[3];
      }
      foxglove.publish_debug(dbg);
    }
```

> 关键顺序：`publish_image` 必须在 `cv::resize(img, img, {}, 0.5, 0.5)` **之前**调用，保证装甲板框坐标与图像分辨率一致。

---

## 4. 编译

```bash
cd ~/桌面/DGE_vision_26
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j"$(nproc)"
```

首次会编译 SDK（约 12 个源文件），之后增量。产物：`build/standard`。

> 若改动过 `CMakeLists.txt`，`make` 会自动重跑 `cmake`；若缓存异常，删掉 `build/CMakeCache.txt` 后重新 `cmake ..`。

---

## 5. 运行

```bash
# 终端 1：先启动 Daedalus 仿真器（必须从仓库根 cargo run）
cd ~/桌面/simulation/bevy_robomaster_simulator-master
source ~/.cargo/env && cargo run --release

# 终端 2：启动 DGE（注意 OpenVINO 的 LD_LIBRARY_PATH）
cd ~/桌面/DGE_vision_26
export LD_LIBRARY_PATH=/opt/intel/openvino_2024.6.0/runtime/lib/intel64:$LD_LIBRARY_PATH
./build/standard configs/daedalus.yaml
```

启动后 DGE 日志应出现：

```
[info] foxglove: WebSocket server on ws://0.0.0.0:8765
```

仿真器窗口里按 **F5** 开启自瞄订阅，按 **IJKL** 把敌方假人开进视野。

---

## 6. Foxglove 使用说明

### 6.1 连接

1. 打开 **Foxglove Studio**（桌面版，或浏览器 `https://studio.foxglove.dev`）。
2. `Open connection` → 选 **Foxglove WebSocket** → URL 填 `ws://localhost:8765`
   （子协议 `foxglove.sdk.v1` 由 Studio 自动协商，无需手填）。
3. 连接后左侧应能看到三个话题：`/camera/image`、`/camera/annotations`、`/dge/debug`。

> 远程机器：把 `localhost` 换成 DGE 所在主机 IP（服务器监听 `0.0.0.0`）。

### 6.2 推荐面板布局

| 面板 | 配置 |
|---|---|
| **Image** | Topic 选 `/camera/image`；在该面板的 **Annotations** 设置里选 `/camera/annotations`（识别框会叠加到图上） |
| **Plot** | Topic 选 `/dge/debug`，勾 `fps`、`yaw`、`pitch`、`aim_yaw` 等字段画曲线 |
| **Raw Messages** | Topic 选 `/dge/debug`，查看完整数值（含 `ekf[]`） |

### 6.3 使用小贴士

- 图像默认限频约 **15fps**（`publish_image` 内部节流），降低带宽；需要更高帧率可改 `foxglove.cpp` 里的 `66000000ULL`。
- **没有客户端连接时，DGE 跳过图像编码**（`has_clients()` 判断），不影响算法性能。
- 曲线字段可扩展：在 `standard.cpp` 的 `dbg` JSON 里加键，Foxglove 的 Plot 面板即可直接选到。

---

## 7. 常见问题（排错）

| 现象 | 原因 / 处理 |
|---|---|
| 编译报 `foxglove/websocket.hpp: No such file` | SDK 未解压到 `3dparty/foxglove/`。按 3.1 重新放置。 |
| 链接报 `undefined reference to foxglove_gateway_*` | 把 `remote_access.cpp` 也编进了 SDK。**只编第 3.4 节列出的 12 个源文件**，不要用 `src/*.cpp` 通配。 |
| 链接报重复符号 | `tools/CMakeLists.txt` 里又加了 `foxglove.cpp`。删掉，改由 `dge_foxglove` 目标编译。 |
| Foxglove 连接返回 `400 Bad Request: Missing expected sec-websocket-protocol header` | 客户端未带子协议。用 Foxglove Studio 连接即可；手写客户端须带 `Sec-WebSocket-Protocol: foxglove.sdk.v1`。 |
| 连上但没有图像 | 仿真器没在跑 / DGE 没收到帧 / 仿真器没按 F5；且发图需要**至少一个 Foxglove 客户端已订阅** `/camera/image`。 |
| 看不见识别框 | Image 面板未在 Annotations 里选 `/camera/annotations`；或该帧没识别到装甲板。 |
| 8765 端口被占用 | 若同时跑另一个也用 8765 的程序会冲突。改 `standard.cpp` 里 `FoxgloveServer foxglove(8765)` 的端口，或关掉另一端。 |

---

## 8. 卸载 / 回退

1. 删除 `src/standard.cpp` 中三处 Foxglove 代码（include、构造、发布块）。
2. 从 `standard` 的 `target_link_libraries` 去掉 `dge_foxglove`。
3. 删除顶层 `CMakeLists.txt` 的 Foxglove 段落（`foxglove_cpp` + `dge_foxglove`）。
4. 删除 `tools/foxglove.hpp`、`tools/foxglove.cpp`、`3dparty/foxglove/`。
5. 重新 `cmake .. && make`。

---

## 9. 后续扩展（可选）

| 需求 | 做法 |
|---|---|
| **MCAP 录制**（离线回放） | SDK 自带 `foxglove::McapWriter`。在 `Impl` 里加一个 writer（`McapWriterOptions{ .context = ..., .path = "record/dge.mcap" }`）即可，与 WebSocket 可并存。 |
| **3D 场景**（装甲板/目标 3D、坐标系） | 用 `foxglove::messages::SceneUpdateChannel`（`SceneEntity`/`CubePrimitive` 等）；DGE 已有 `solver` 的世界系坐标可用。 |
| **相机内参** | 用 `CameraCalibrationChannel` 发布内参，Image 面板可直接做图像→3D 对齐。 |
| **日志进 Foxglove** | SDK 有 `Log` 消息类型；可仿照成熟项目把 spdlog 日志也推到 Log 面板。 |

---

## 附：验证记录（2026-10-11）

- `cmake .. && make`：`foxglove_cpp`、`dge_foxglove`、`standard` 全部构建成功。
- 运行 `./build/standard configs/daedalus.yaml`：日志打印 `foxglove: WebSocket server on ws://0.0.0.0:8765`。
- 原始 WebSocket 客户端握手：`HTTP/1.1 101 Switching Protocols`，`serverInfo` 返回 `{"op":"serverInfo","name":"DGE_vision_26",...,"fg-library":"foxglove-sdk-cpp/v0.22.0"}`。
- 测试时仿真器（`target/release/daedalus`）正在运行，DGE 实际处理真实帧（tracker 正常），证明闭环与 SDK 服务可同时工作。
