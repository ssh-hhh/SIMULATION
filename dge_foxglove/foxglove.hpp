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
//
// 连接方式：Foxglove Studio -> Open connection -> Foxglove WebSocket -> ws://<本机IP>:8765
//
// 通道：
//   /camera/image       foxglove.CompressedImage   (JPEG 图像)
//   /camera/annotations foxglove.ImageAnnotations  (装甲板识别框)
//   /dge/debug          json                        (fps/yaw/pitch/shoot/EKF...，Plot 面板画曲线)
//
// 实现细节全部藏在 .cpp 里（pimpl），使用方无需包含 SDK 头文件。
class FoxgloveServer
{
public:
  explicit FoxgloveServer(int port = 8765, std::string name = "DGE_vision_26");
  ~FoxgloveServer();

  FoxgloveServer(const FoxgloveServer &) = delete;
  FoxgloveServer & operator=(const FoxgloveServer &) = delete;

  // 是否有客户端连接（无客户端时可跳过高开销的图像编码）
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
