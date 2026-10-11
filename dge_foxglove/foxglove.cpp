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
