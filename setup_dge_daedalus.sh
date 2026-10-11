#!/usr/bin/env bash
# DGE_vision_26 一键接入 Daedalus 仿真器
# 用法: bash setup_dge_daedalus.sh [DGE目录]（默认 ~/桌面/DGE_vision_26）
# 前提: ~/桌面/simulation/bevy_robomaster_simulator-master 存在（提供协议头文件）
# 幂等：重复运行安全（已打过的补丁会跳过）
# 详细原理见 ~/桌面/simulation/环境配置与使用指南.md 第 10 节

set -e
DGE_DIR="${1:-$HOME/桌面/DGE_vision_26}"
SIM_DIR="$HOME/桌面/simulation/bevy_robomaster_simulator-master"
TALOS_DIR="$HOME/桌面/simulation/talos26-master"
# talos26-master 可能已被删除；协议头文件的备用来源（git clone 备份）
TALOS_FALLBACK="$HOME/.zcode/workspace/default/talos26"

[ -d "$DGE_DIR/io" ] || { echo "DGE 目录不对: $DGE_DIR"; exit 1; }

# 定位协议头文件来源：优先 talos26-master，其次备份副本，最后用 DGE 里已有的
find_src() {
  local f="$1"
  for d in "$TALOS_DIR" "$TALOS_FALLBACK"; do
    if [ -f "$d/crates/hardware_daedalus/src/$f" ]; then
      echo "$d/crates/hardware_daedalus/src/$f"; return
    fi
  done
  if [ -f "$DGE_DIR/io/daedalus/$f" ]; then echo "$DGE_DIR/io/daedalus/$f"; return; fi
  echo ""
}

cd "$DGE_DIR"

# ---------- 1. 协议头文件 ----------
mkdir -p io/daedalus
SHM_LAYOUT_SRC="$(find_src shm_layout.hpp)"
SHM_TRIPLE_SRC="$(find_src shm_triple_buffer.hpp)"
if [ -n "$SHM_LAYOUT_SRC" ] && [ "$SHM_LAYOUT_SRC" != "$DGE_DIR/io/daedalus/shm_layout.hpp" ]; then
  cp "$SHM_LAYOUT_SRC" io/daedalus/shm_layout.hpp
fi
if [ -n "$SHM_TRIPLE_SRC" ] && [ "$SHM_TRIPLE_SRC" != "$DGE_DIR/io/daedalus/shm_triple_buffer.hpp" ]; then
  cp "$SHM_TRIPLE_SRC" io/daedalus/shm_triple_buffer.hpp
fi
[ -f io/daedalus/shm_layout.hpp ] && [ -f io/daedalus/shm_triple_buffer.hpp ] \
  || { echo "找不到 shm_layout.hpp / shm_triple_buffer.hpp（talos26 与备份均缺失）"; exit 1; }

# ---------- 2. 新增文件（完整写入） ----------
cat > io/daedalus/shm_client.hpp <<'SHM_CLIENT'
#ifndef IO__DAEDALUS__SHM_CLIENT_HPP
#define IO__DAEDALUS__SHM_CLIENT_HPP

// Daedalus 仿真器共享内存客户端（C++17 精简版）
// 协议契约与 bevy_robomaster_simulator/crates/talos-ipc/src/layout.rs (SHM_VERSION=2) 一致。
// 原始参考实现: talos26/crates/hardware_daedalus/src/shm_client.hpp (C++23 std::expected 版)

#include "shm_layout.hpp"
#include "shm_triple_buffer.hpp"

#include <chrono>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <optional>
#include <opencv2/core.hpp>
#include <stdexcept>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace ipc
{

// 与 Rust 侧 static_assert 对齐的布局常量
static_assert(sizeof(ImageTripleBuffer) == 192);
static_assert(sizeof(PoseTripleBuffer) == 256);
static_assert(sizeof(GimbalTripleBuffer) == 192);
static_assert(sizeof(CameraInfo) == 128);
static_assert(sizeof(ShmHeader) == 64);
static_assert(sizeof(ImageMeta) == 32);
static_assert(sizeof(PoseMeta) == 64);
static_assert(sizeof(GimbalCmd) == 32);
static_assert(sizeof(ShmMetaRegion) == 3712);

class ShmClient
{
public:
  struct ImageFrame
  {
    cv::Mat image;         // 零拷贝 cv::Mat（指向共享内存，下次 recv 前有效）
    uint64_t seq;
    uint64_t timestamp_ns;
  };

  struct Pose
  {
    double x, y, z;
    double qw, qx, qy, qz;
    uint64_t frame_seq;
    uint64_t timestamp_ns;
  };

  ShmClient()
  {
    meta_fd_ = open_file("/tmp/" + std::string(SHM_NAME_META));
    map_region(meta_fd_, sizeof(ShmMetaRegion), meta_);

    pool_fd_ = open_file("/tmp/" + std::string(SHM_NAME_IMAGE_POOL));
    map_region(pool_fd_, IMAGE_POOL_SIZE, pool_);

    auto * meta = reinterpret_cast<ShmMetaRegion *>(meta_);
    if (meta->header.magic != SHM_MAGIC)
      throw std::runtime_error("daedalus shm magic mismatch (版本不一致?)");
    if (meta->header.version != SHM_VERSION)
      throw std::runtime_error("daedalus shm version mismatch (版本不一致?)");
  }

  ~ShmClient()
  {
    munmap(meta_, sizeof(ShmMetaRegion));
    munmap(pool_, IMAGE_POOL_SIZE);
    close(meta_fd_);
    close(pool_fd_);
  }

  const CameraInfo & camera_info() const
  {
    return reinterpret_cast<const ShmMetaRegion *>(meta_)->camera_info;
  }

  bool is_producer_alive(const uint64_t timeout_ns = 1'000'000'000) const
  {
    auto * meta = reinterpret_cast<const ShmMetaRegion *>(meta_);
    return now_ns() - meta->header.heartbeat_ns < timeout_ns;
  }

  // 尝试取最新图像帧（消费后释放 FLAG_NEW，发布端才能发下一帧）
  std::optional<ImageFrame> recv_image()
  {
    auto * meta = reinterpret_cast<ShmMetaRegion *>(meta_);
    ImageOps ops(&meta->image);
    const auto slot = ops.borrow();
    if (!slot) return std::nullopt;

    const auto & img_meta = **slot;
    uint8_t * img_data = reinterpret_cast<uint8_t *>(pool_) + img_meta.buffer_id * IMAGE_SIZE;

    int cv_type = CV_8UC3;  // 默认 RGB8 (format 0)
    if (img_meta.format == 1) cv_type = CV_8UC3;         // BGR8
    else if (img_meta.format == 2) cv_type = CV_8UC1;    // GRAY8

    ImageFrame frame;
    frame.image = cv::Mat(img_meta.height, img_meta.width, cv_type);
    std::memcpy(frame.image.data, img_data, IMAGE_SIZE);
    frame.seq = img_meta.seq;
    frame.timestamp_ns = img_meta.timestamp_ns;
    return frame;
  }

  // 取指定位姿通道的最新数据（不阻塞）
  std::optional<Pose> recv_pose(const PoseIndex index)
  {
    auto * meta = reinterpret_cast<ShmMetaRegion *>(meta_);
    PoseOps ops(&meta->poses[static_cast<int>(index)]);
    const auto slot = ops.borrow();
    if (!slot) return std::nullopt;

    const auto & p = **slot;
    Pose pose;
    pose.x = p.position[0];
    pose.y = p.position[1];
    pose.z = p.position[2];
    pose.qw = p.quaternion[0];
    pose.qx = p.quaternion[1];
    pose.qy = p.quaternion[2];
    pose.qz = p.quaternion[3];
    pose.frame_seq = p.frame_seq;
    pose.timestamp_ns = p.timestamp_ns;
    return pose;
  }

  // 发送云台命令：绝对角度制（度），fire_advice 为开火建议
  void send_gimbal_cmd(
    const float yaw_deg, const float pitch_deg, const float distance_m,
    const bool fire_advice)
  {
    auto * meta = reinterpret_cast<ShmMetaRegion *>(meta_);
    GimbalOps ops(&meta->gimbal_cmd);
    auto & cmd = ops.borrow_mut();

    cmd.timestamp_ns = now_ns();
    cmd.yaw_deg = yaw_deg;
    cmd.pitch_deg = pitch_deg;
    cmd.distance_m = distance_m;
    cmd.fire_advice = fire_advice ? 1 : 0;

    ops.publish();
  }

private:
  // 仿真器的心跳用 UNIX_EPOCH 纳秒（SystemTime），必须用同一时钟域比较
  static uint64_t now_ns()
  {
    return static_cast<uint64_t>(
      std::chrono::system_clock::now().time_since_epoch().count());
  }

  static int open_file(const std::string & path)
  {
    int fd = open(path.c_str(), O_RDWR);
    if (fd < 0) throw std::runtime_error("无法打开共享内存 " + path + "（仿真器没在运行?）");
    return fd;
  }

  static void map_region(int fd, size_t size, void * & out)
  {
    void * p = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (p == MAP_FAILED) throw std::runtime_error("mmap 失败");
    out = p;
  }

  int meta_fd_ = -1;
  int pool_fd_ = -1;
  void * meta_ = nullptr;
  void * pool_ = nullptr;
};

}  // namespace ipc

#endif  // IO__DAEDALUS__SHM_CLIENT_HPP
SHM_CLIENT

cat > io/daedalus/shared.hpp <<'SHARED'
#ifndef IO__DAEDALUS__SHARED_HPP
#define IO__DAEDALUS__SHARED_HPP

// DaedalusCamera 与 DaedalusCBoard 共享的进程级单例：
// 一个 ShmClient 客户端 + IMU 四元数队列 + 符号/坐标系映射参数。
// 图像与位姿都由 DaedalusCamera::read() 驱动接收，DaedalusCBoard 只读队列、发指令。

#include <Eigen/Geometry>
#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include "shm_client.hpp"
#include "tools/logger.hpp"
#include <yaml-cpp/yaml.h>

namespace io
{

struct DaedalusImuSample
{
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point timestamp;
};

struct DaedalusShared
{
  std::shared_ptr<ipc::ShmClient> client;
  std::mutex imu_mutex;
  std::deque<DaedalusImuSample> imu_queue;
  const size_t imu_queue_capacity = 5000;

  // command.yaw/pitch (rad, DGE世界系) → GimbalCmd (deg, 仿真器世界系) 的符号映射
  double yaw_sign = 1.0;
  double pitch_sign = -1.0;  // talos 同款：仿真器 pitch_deg 与 DGE pitch 反号

  // Bevy(Y-up) → DGE(Z-up, X-forward) 坐标系变换: q' = left * q * right.conjugate()
  // 默认 left = right = (0.5, 0.5, 0.5, 0.5)，即绕 (1,1,1) 轴 120° 的循环轴变换
  Eigen::Quaterniond imu_left = Eigen::Quaterniond(0.5, 0.5, 0.5, 0.5);
  Eigen::Quaterniond imu_right = Eigen::Quaterniond(0.5, 0.5, 0.5, 0.5);

  // 最近一次收到的云台位姿（Bevy 系原始四元数），位姿通道无新数据时沿用
  Eigen::Quaterniond last_gimbal_q = Eigen::Quaterniond::Identity();

  // imu_at() 的游标状态（与 CBoard 的 data_ahead_/data_behind_ 等价）
  DaedalusImuSample last_imu_sample{
    Eigen::Quaterniond::Identity(), std::chrono::steady_clock::time_point::min()};

  static DaedalusShared & instance(const std::string & config_path)
  {
    static DaedalusShared shared;
    static bool initialized = false;
    if (!initialized) {
      initialized = true;
      try {
        shared.client = std::make_shared<ipc::ShmClient>();
      } catch (const std::exception & e) {
        tools::logger()->error("[Daedalus] {}", e.what());
        exit(1);
      }
      tools::logger()->info("[Daedalus] shm connected (producer alive: {})",
                            shared.client->is_producer_alive());

      auto yaml = YAML::LoadFile(config_path);
      if (yaml["daedalus"]) {
        auto d = yaml["daedalus"];
        if (d["yaw_sign"]) shared.yaw_sign = d["yaw_sign"].as<double>();
        if (d["pitch_sign"]) shared.pitch_sign = d["pitch_sign"].as<double>();
        if (d["imu_left"]) {
          auto v = d["imu_left"].as<std::vector<double>>();
          shared.imu_left = Eigen::Quaterniond(v[0], v[1], v[2], v[3]).normalized();
        }
        if (d["imu_right"]) {
          auto v = d["imu_right"].as<std::vector<double>>();
          shared.imu_right = Eigen::Quaterniond(v[0], v[1], v[2], v[3]).normalized();
        }
      }
    }
    return shared;
  }

  void push_imu(const Eigen::Quaterniond & q_bevy)
  {
    Eigen::Quaterniond q =
      (imu_left * q_bevy * imu_right.conjugate()).normalized();
    std::lock_guard<std::mutex> lock(imu_mutex);
    if (imu_queue.size() >= imu_queue_capacity) imu_queue.pop_front();
    imu_queue.push_back({q, std::chrono::steady_clock::now()});
  }
};

}  // namespace io

#endif  // IO__DAEDALUS__SHARED_HPP
SHARED

cat > io/daedalus_camera.hpp <<'CAMERA_HPP'
#ifndef IO__DAEDALUS_CAMERA_HPP
#define IO__DAEDALUS_CAMERA_HPP

#include "io/camera.hpp"

namespace io
{

// 从 Daedalus 仿真器共享内存读取图像的 CameraBase 实现。
// 每次 read() 消费一帧 RGB 并转成 BGR（OpenCV 惯例），同时把该帧对应的
// 云台位姿四元数（经坐标系变换）压入共享 IMU 队列，供 DaedalusCBoard::imu_at 对齐。
class DaedalusCamera : public CameraBase
{
public:
  explicit DaedalusCamera(const std::string & config_path);

  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;
};

}  // namespace io

#endif  // IO__DAEDALUS_CAMERA_HPP
CAMERA_HPP

cat > io/daedalus_camera.cpp <<'CAMERA_CPP'
#include "io/daedalus_camera.hpp"

#include "io/daedalus/shared.hpp"
#include "tools/logger.hpp"

#include <chrono>
#include <stdexcept>
#include <thread>

namespace io
{

DaedalusCamera::DaedalusCamera(const std::string & config_path)
{
  auto & shared = DaedalusShared::instance(config_path);
  const auto & ci = shared.client->camera_info();
  tools::logger()->info(
    "[DaedalusCamera] {}x{} fx={:.2f} fy={:.2f} cx={:.2f} cy={:.2f}", ci.width, ci.height, ci.fx,
    ci.fy, ci.cx, ci.cy);
}

void DaedalusCamera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  auto & shared = DaedalusShared::instance("");
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);

  while (true) {
    // 消费全部位姿通道：仿真器的图像发布握手中，gimbal/odom/muzzle/camera 四个
    // 位姿通道的 FLAG_NEW 都必须被清掉才会发布下一帧图像（见 talos-ipc
    // publisher.rs 的 synchronized_frame_consumed）。不消费会导致死锁。
    for (int i = 0; i <= 4; ++i) {
      auto index = static_cast<ipc::PoseIndex>(i);
      if (auto pose = shared.client->recv_pose(index)) {
        if (index == ipc::POSE_GIMBAL) {
          shared.last_gimbal_q =
            Eigen::Quaterniond(pose->qw, pose->qx, pose->qy, pose->qz);
        }
      }
    }

    if (auto frame = shared.client->recv_image()) {
      // 仿真器发布 RGB8，转 BGR 供 OpenCV/识别器使用
      cv::cvtColor(frame->image, img, cv::COLOR_RGB2BGR);
      timestamp = std::chrono::steady_clock::now();

      shared.push_imu(shared.last_gimbal_q);
      return;
    }

    if (!shared.client->is_producer_alive()) {
      tools::logger()->warn("[DaedalusCamera] 仿真器心跳丢失，等待恢复...");
    }
    if (std::chrono::steady_clock::now() > deadline) {
      // 不抛异常：置空图像让上层跳过本帧（standard.cpp 会 continue）
      tools::logger()->error("[DaedalusCamera] 等待图像超时（仿真器在运行吗？）");
      img.release();
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

}  // namespace io
CAMERA_CPP

cat > io/daedalus_cboard.hpp <<'CBOARD_HPP'
#ifndef IO__DAEDALUS_CBOARD_HPP
#define IO__DAEDALUS_CBOARD_HPP

#include <algorithm>
#include <cmath>

#include "io/cboard.hpp"
#include "io/daedalus/shared.hpp"

namespace io
{

// CBoard 的仿真替身：不连串口，通过共享内存与 Daedalus 仿真器通信。
// - imu_at(): 从 IMU 队列插值（队列由 DaedalusCamera::read() 按帧填充）
// - send():   把 DGE 世界系的绝对 yaw/pitch (rad) 换算成度发给仿真器，
//             fire_advice 来自 command.shoot
class DaedalusCBoard : public CBoardBase
{
public:
  explicit DaedalusCBoard(const std::string & config_path)
  {
    DaedalusShared::instance(config_path);  // 确保单例已初始化
  }

  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp) override
  {
    auto & shared = DaedalusShared::instance("");

    DaedalusImuSample data_ahead = shared.last_imu_sample;
    DaedalusImuSample data_behind = shared.last_imu_sample;

    while (true) {
      DaedalusImuSample temp;
      bool has_data = false;
      {
        std::lock_guard<std::mutex> lock(shared.imu_mutex);
        if (!shared.imu_queue.empty()) {
          temp = shared.imu_queue.front();
          shared.imu_queue.pop_front();
          has_data = true;
        }
      }
      if (!has_data) break;

      data_behind = temp;
      if (data_behind.timestamp > timestamp) break;
      data_ahead = data_behind;
    }
    shared.last_imu_sample = data_ahead;

    // slerp 插值（与 CBoard::imu_at 相同的逻辑）
    const Eigen::Quaterniond & q_a = data_ahead.q;
    const Eigen::Quaterniond & q_b = data_behind.q;
    auto t_a = data_ahead.timestamp;
    auto t_b = data_behind.timestamp;
    auto t_c = timestamp;

    if (t_a == t_b) return q_a;

    double k = double((t_c - t_a).count()) / double((t_b - t_a).count());
    k = std::max(0.0, std::min(1.0, k));
    return q_a.slerp(k, q_b);
  }

  void send(Command command) const override
  {
    auto & shared = DaedalusShared::instance("");
    constexpr double kRadToDeg = 180.0 / M_PI;
    shared.client->send_gimbal_cmd(
      static_cast<float>(shared.yaw_sign * command.yaw * kRadToDeg),
      static_cast<float>(shared.pitch_sign * command.pitch * kRadToDeg),
      static_cast<float>(command.horizon_distance), command.shoot);
  }
};

}  // namespace io

#endif  // IO__DAEDALUS_CBOARD_HPP
CBOARD_HPP

# ---------- 2.5 Foxglove 可视化（官方 C++ SDK，无 ROS） ----------
FOX_DIR="$HOME/桌面/simulation/dge_foxglove"
FOX_SDK_ZIP="$HOME/桌面/simulation/foxglove-v0.23.0-cpp-x86_64-unknown-linux-gnu.zip"
if [ -f "$FOX_DIR/foxglove.cpp" ]; then
  mkdir -p tools
  cp "$FOX_DIR/foxglove.hpp" tools/foxglove.hpp
  cp "$FOX_DIR/foxglove.cpp" tools/foxglove.cpp
  echo "[ok] tools/foxglove.{hpp,cpp}"
else
  echo "[warn] 未找到 $FOX_DIR/foxglove.cpp，跳过 Foxglove 模块"
fi
# 官方 Foxglove C++ SDK（vendored，C++17 兼容）
if [ -d 3dparty/foxglove/include ]; then
  echo "[skip] 3dparty/foxglove 已存在"
elif [ -f "$FOX_SDK_ZIP" ]; then
  mkdir -p 3dparty
  unzip -oq "$FOX_SDK_ZIP" -d 3dparty/
  echo "[ok] 解压 Foxglove SDK -> 3dparty/foxglove"
else
  echo "[warn] 未找到 $FOX_SDK_ZIP，无法安装 Foxglove SDK（编译会失败）"
fi

# ---------- 3. 打补丁（幂等） ----------
python3 <<'PATCH'
import pathlib, sys

def patch(path, old, new, tag):
    p = pathlib.Path(path)
    text = p.read_text()
    if tag in text:
        print(f"[skip] {path} 已打过补丁")
        return
    if old not in text:
        print(f"[FAIL] {path} 找不到目标代码片段（上游代码变了？）")
        sys.exit(1)
    p.write_text(text.replace(old, new, 1))
    print(f"[ok] {path}")

def patch_once(path, old, new):
    # 以"旧片段消失"为幂等依据：旧片段还在就替换，否则跳过
    p = pathlib.Path(path)
    text = p.read_text()
    if old not in text:
        print(f"[skip] {path} 已打过补丁")
        return
    p.write_text(text.replace(old, new, 1))
    print(f"[ok] {path}")

# 3.1 camera.cpp: include + daedalus 分支
patch("io/camera.cpp",
      '#include "hikrobot/hikrobot.hpp"',
      '#include "daedalus_camera.hpp"\n#include "hikrobot/hikrobot.hpp"',
      'daedalus_camera.hpp')
patch("io/camera.cpp",
      '''  else {
    throw std::runtime_error("Unknow camera_name: " + camera_name + "!");
  }''',
      '''  else if (camera_name == "daedalus") {
    camera_ = std::make_unique<DaedalusCamera>(config_path);
  }

  else {
    throw std::runtime_error("Unknow camera_name: " + camera_name + "!");
  }''',
      '"daedalus"')

# 3.2 cboard.hpp: 抽出 CBoardBase 基类
patch("io/cboard.hpp",
      '''class CBoard
{
public:
  double bullet_speed;
  Mode mode;
  // ShootMode shoot_mode;
  double ft_angle;

  CBoard(const std::string & config_path);

  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);

  void send(Command command) const;''',
      '''// 云台通信的抽象基类：真机用 CBoard（串口），仿真用 DaedalusCBoard（共享内存）。
class CBoardBase
{
public:
  CBoardBase() = default;
  virtual ~CBoardBase() = default;

  double bullet_speed = 0;
  Mode mode = Mode::idle;
  double ft_angle = 0;

  virtual Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp) = 0;
  virtual void send(Command command) const = 0;
};

class CBoard : public CBoardBase
{
public:
  CBoard(const std::string & config_path);

  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp) override;

  void send(Command command) const override;''',
      'class CBoardBase')

# 3.2b cboard.cpp: 基类字段不能出现在派生类初始化列表
patch_once("io/cboard.cpp",
      ''': mode(Mode::idle),
  // shoot_mode(ShootMode::left_shoot),
  // bullet_speed(0),
  ft_angle(0),
  // queue_ initialization removed (using default deque constructor)''',
      ''': // shoot_mode(ShootMode::left_shoot),
  // bullet_speed(0),
  // queue_ initialization removed (using default deque constructor)''')

# 3.3 standard.cpp: include + 运行时选择云台后端
patch("src/standard.cpp",
      '''#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>''',
      '''#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>
#include <thread>
#include <yaml-cpp/yaml.h>''',
      '<memory>')
patch("src/standard.cpp",
      '#include "io/cboard.hpp"',
      '#include "io/cboard.hpp"\n#include "io/daedalus_cboard.hpp"',
      'daedalus_cboard.hpp')
patch("src/standard.cpp",
      '''  io::CBoard cboard(config_path); ''',
      '''  // 仿真时在 yaml 里写 cboard_name: daedalus，真机不写或写 cboard
  std::unique_ptr<io::CBoardBase> cboard_ptr;
  {
    auto yaml = YAML::LoadFile(config_path);
    auto cboard_name =
      yaml["cboard_name"] ? yaml["cboard_name"].as<std::string>() : "cboard";
    if (cboard_name == "daedalus")
      cboard_ptr = std::make_unique<io::DaedalusCBoard>(config_path);
    else
      cboard_ptr = std::make_unique<io::CBoard>(config_path);
  }
  auto & cboard = *cboard_ptr;''',
      'cboard_ptr')

# 3.4 io/CMakeLists.txt: 加源文件 + include 目录
patch("io/CMakeLists.txt",
      '''    camera.cpp
    cboard.cpp''',
      '''    camera.cpp
    cboard.cpp
    daedalus_camera.cpp''',
      'daedalus_camera.cpp')
patch("io/CMakeLists.txt",
      '''gimbal/gimbal.cpp
)''',
      '''gimbal/gimbal.cpp
)

# Daedalus 仿真器 IPC 适配层（协议头在 io/daedalus/，header-only，无额外链接）
target_include_directories(io PUBLIC daedalus)''',
      'target_include_directories(io PUBLIC daedalus)')

# 3.5 Foxglove: 顶层 CMakeLists.txt 加入官方 SDK 目标 + DGE 桥接库
patch("CMakeLists.txt",
      '''include_directories(${PROJECT_SOURCE_DIR})''',
      '''include_directories(${PROJECT_SOURCE_DIR})

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
target_link_libraries(dge_foxglove PUBLIC foxglove_cpp ${OpenCV_LIBS})''',
      'foxglove_cpp')
patch("CMakeLists.txt",
      'target_link_libraries(standard ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)',
      'target_link_libraries(standard ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io dge_foxglove)',
      'tools io dge_foxglove')

# 3.6 standard.cpp: Foxglove 服务器 + 发布图像/识别框/调试数据
patch("src/standard.cpp",
      '#include "tools/plotter.hpp"',
      '#include "tools/foxglove.hpp"\n#include "tools/plotter.hpp"',
      'tools/foxglove.hpp')
patch("src/standard.cpp",
      '''  tools::Plotter plotter;''',
      '''  tools::Plotter plotter;
  tools::FoxgloveServer foxglove(8765);''',
      'FoxgloveServer foxglove')
patch("src/standard.cpp",
      '''    for(const auto& armor : armors) {
        for(int i=0; i<4; i++) 
            cv::line(img, armor.points[i], armor.points[(i+1)%4], {0,255,0}, 2);
    }''',
      '''    for(const auto& armor : armors) {
        for(int i=0; i<4; i++) 
            cv::line(img, armor.points[i], armor.points[(i+1)%4], {0,255,0}, 2);
    }

    // Foxglove：推送（原始分辨率）图像 + 装甲板识别框
    foxglove.publish_image(img);
    foxglove.publish_annotations(armors);''',
      'foxglove.publish_annotations')
patch("src/standard.cpp",
      '''    std::string text_shoot = fmt::format("Shoot: {}", command.shoot ? 1 : 0);
    cv::putText(img, text_shoot, cv::Point(10, 60), font, scale, cv::Scalar(0, 0, 255), thickness);''',
      '''    std::string text_shoot = fmt::format("Shoot: {}", command.shoot ? 1 : 0);
    cv::putText(img, text_shoot, cv::Point(10, 60), font, scale, cv::Scalar(0, 0, 255), thickness);

    // Foxglove 调试曲线（Plot 面板可直接选字段）
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
        dbg["aim_x"] = a[0];
        dbg["aim_y"] = a[1];
        dbg["aim_z"] = a[2];
        dbg["aim_yaw"] = a[3];
      }
      foxglove.publish_debug(dbg);
    }''',
      'foxglove.publish_debug')
PATCH

# ---------- 4. daedalus.yaml ----------
cat > configs/daedalus.yaml <<'YAML'
# Daedalus 仿真闭环配置（基于 standard3.yaml，内参/外参换为仿真器值）
# 用法: ./build/standard configs/daedalus.yaml   （先启动 Daedalus 仿真器）

enemy_color: "blue"   # 仿真场内可见敌方为蓝方（装甲板灯条蓝色）；红方改回 "red"

#####-----神经网络参数-----#####
yolo_name: yolov5
classify_model: assets/tiny_resnet.onnx
yolo11_model_path: assets/yolo11.xml
yolov8_model_path: assets/yolov8.xml
yolov5_model_path: assets/yolov5.xml
device: CPU
min_confidence: 0.8
use_traditional: false

#####-----ROI-----#####
roi:
  x: 420
  y: 50
  width: 600
  height: 600

use_roi: false

#####-----传统方法参数-----#####
threshold: 150
max_angle_error: 45 # degree
min_lightbar_ratio: 1.5
max_lightbar_ratio: 20
min_lightbar_length: 8
min_armor_ratio: 1
max_armor_ratio: 5
max_side_ratio: 1.5
max_rectangular_error: 25 # degree

#####-----tracker参数-----#####
min_detect_count: 5 # 最小确认帧数
max_temp_lost_count: 15 #最大暂存丢失帧数
outpost_max_temp_lost_count: 75

#####-----aimer参数-----#####
# 仿真下真机标定偏置清零，方向不对时优先调 daedalus.yaw_sign/pitch_sign
yaw_offset: 0
pitch_offset: 0
comming_angle: 60 # degree 切入角
leaving_angle: 20 # degree 切出角
decision_speed: 7 # rad/s 决策速度
high_speed_delay_time: 0.05 # s
low_speed_delay_time: 0.05 # s

#####-----shooter参数-----#####
first_tolerance: 3 # 近距离射击容差，degree
second_tolerance: 2 # 远距离射击容差，degree
judge_distance: 2 #距离判断阈值
auto_fire: true # 是否由自瞄控制射击

#####-----相机选择-----#####
camera_name: "daedalus"   # 走共享内存接 Daedalus 仿真器
exposure_ms: 3.0          # daedalus 模式下不使用，但工厂函数要求该键存在
gain: 16.0
vid_pid: "2bdf:0001"

# 仿真器相机内参（1440x1080，完美针孔，由 probe2 读取自共享内存 camera_info）
camera_matrix: [1303.675283386667, 0, 720, 0, 1303.675283386667, 540, 0, 0, 1]
distort_coeffs: [0, 0, 0, 0, 0]

R_gimbal2imubody: [1, 0, 0, 0, 1, 0, 0, 0, 1]

# 仿真器相机相对云台的安装关系（Bevy 坐标系实测换算，误差影响极小）
R_camera2gimbal: [0, 0, 1, -1, 0, 0, 0, -1, 0]
t_camera2gimbal: [0.196, 0.06, 0.002]

#####-----云台后端-----#####
cboard_name: "daedalus"   # 走共享内存发云台指令（真机删掉此行用串口 CBoard）

# Daedalus 适配参数：实测仿真器 yaw 正方向为向右转，与 DGE 相反，故 yaw_sign=-1（2026-10-11 标定）
daedalus:
  yaw_sign: -1
  pitch_sign: -1
  # imu_left: [0.5, 0.5, 0.5, 0.5]    # Bevy→DGE 轴变换四元数（一般不用动）
  # imu_right: [0.5, 0.5, 0.5, 0.5]
YAML

echo ""
echo "===== 全部就绪，开始编译 ====="
mkdir -p build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release > /dev/null 2>&1 && make -j12 2>&1 | grep -E "error|Built target standard" | tail -5
