#include "runtime/boot.hpp"

#include "L1_sensor/output_interface.hpp"
#include "L2_perception/armor/readback_roi.hpp"
#include "L2_perception/armor/systems.hpp"
#include "L2_perception/ldm/ldm_systems.hpp"
#include "L2_perception/rune/rune_systems.hpp"
#include "L3_estimation/energy_meter/energy_meter_systems.hpp"
#include "L3_estimation/ldm_naive/ldm_naive_systems.hpp"
#include "L3_estimation/tracker_systems.hpp"
#include "L4_planning/planning_systems.hpp"
#include "L5_weapon/enhanced/weapon_systems.hpp"
#include "chiral/chiral_collector_system.hpp"
#include "core/armor_types.hpp"
#include "core/channel_topics.hpp"
#include "core/runtime.hpp"
#include "core/trajectory/resource.hpp"
#include "core/types.hpp"
#include "frame.hpp"
#include "quanta/stream_transport.hpp"
#include "runtime/capturer.hpp"
#include "runtime/config_loader.hpp"
#include "runtime/l1_l2_setup.hpp"
#include "runtime/stream_encode.hpp"
#include "scheduler/error_formatter.hpp"
#include <fmt/core.h>
#include <fmt/ranges.h>
#include <numbers>
#include <primitive/overloaded.hpp>
#include <spdlog/spdlog.h>

namespace fcs {

namespace {

inline constexpr auto to_rad(double x) noexcept { return (x / 180.0) * std::numbers::pi_v<double>; }

void init_coordinate_system(fast_tf::CoordinateSystem& system, const RobotExtrinsicConfig& config) {
    constexpr fp_t pi = std::numbers::pi;

    fast_tf::update<fast_tf::odom>(system, {}, 0);
    fast_tf::update<fast_tf::gimbal_yaw_fuxk_frame>(system, {}, 0);

    auto a = config.gimbal_yaw.gimbal_pitch.translation;
    fast_tf::update<fast_tf::gimbal_pitch_fuxk_frame>(
        system,
        fast_tf::EdgeTransform<fast_tf::gimbal_pitch_fuxk_frame>::from_translation(
            a.x(), a.y(), a.z()),
        0);

    auto b = config.gimbal_yaw.gimbal_pitch.camera_link.translation;
    auto c = config.gimbal_yaw.gimbal_pitch.camera_link.rotation();

    fast_tf::update<fast_tf::camera_link_fuxk_frame>(
        system,
        fast_tf::EdgeTransform<fast_tf::camera_link_fuxk_frame>::from_rpy(
            to_rad(c.roll), to_rad(c.pitch), to_rad(c.yaw), b.x(), b.y(), b.z()),
        0);

    fast_tf::update<fast_tf::camera_optical_fuxk_frame>(
        system,
        fast_tf::EdgeTransform<fast_tf::camera_optical_fuxk_frame>::from_rpy(
            -pi / 2.0, 0.0, -pi / 2.0, 0.0, 0.0, 0.0),
        0);

    auto d = config.gimbal_yaw.gimbal_pitch.muzzle_link.translation;
    auto e = config.gimbal_yaw.gimbal_pitch.muzzle_link.rotation();
    fast_tf::update<fast_tf::muzzle_link_fuxk_frame>(
        system,
        fast_tf::EdgeTransform<fast_tf::muzzle_link_fuxk_frame>::from_rpy(
            to_rad(e.roll), to_rad(e.pitch), to_rad(e.yaw), d.x(), d.y(), d.z()),
        0);
}

} // namespace

[[nodiscard]] std::expected<void, std::string>
    boot(talos::Scheduler& scheduler, RuntimeConfig&& config) {
    auto& world = scheduler.world();
    // Insert config as shared resource (for foxglove and other systems)
    world.insert_resource(config.foxglove);

    bool chiral = false;
    // Build the TF buffer in place to avoid moving the large tuple of buffers during startup.
    auto& coordinate_system = world.emplace_resource<fast_tf::CoordinateSystem>();
    std::visit(
        overloaded{
            [&coordinate_system, &world, &chiral](hardware::DirectConfig cfg) {
                init_coordinate_system(coordinate_system, cfg.hardware.extrinsic);
                world.insert_resource(
                    core::trajectory::bullet_speed_data{
                        .bullet_speed = cfg.hardware.mcu->bullet_speed_default});
                chiral = cfg.hardware.chiral;
            },
            [&coordinate_system, &world](hardware::DaedalusConfig cfg) {
                init_coordinate_system(coordinate_system, cfg.extrinsic);
                world.insert_resource(
                    core::trajectory::bullet_speed_data{.bullet_speed = cfg.bullet_speed});
            }},
        config.backend);

    world.insert_resource(config.vision.detect_color);
    static_cast<void>(world.emplace_resource<core::CapabilityState>(
        std::span<const core::Capability>(config.vision.capabilities.get())));
    static_cast<void>(world.emplace_resource<core::FollowingState>());
    world.insert_resource(config.vision.armor->readback_roi);
    world.insert_resource(L2::TrackerReadbackCache{});

    SPDLOG_INFO("color: {}", config.vision.detect_color);
    SPDLOG_INFO("capabilities: [{}]", fmt::join(*config.vision.capabilities, ", "));

    // Setup L1 sensor systems
    auto setup_result = runtime::setup_l1(world, scheduler, config.backend);
    if (!setup_result) {
        return std::unexpected(fmt::format("setup l1: {}", setup_result.error()));
    }

    // Create detector backend
    auto backend_handle_result = L2::create_detector_backend_handle(config.vision.armor.get());
    if (!backend_handle_result) {
        return std::unexpected(
            fmt::format("create detector backend: {}", backend_handle_result.error()));
    }

    SPDLOG_INFO("armor backend: {}", backend_handle_result->name);

    world.insert_resource(std::move(backend_handle_result->backend));
    world.insert_resource(L2::create_pnp_solver(setup_result.value().camera_config));

    // Register trajectory solver resource (used by L4/L5)
    core::trajectory::register_resource(scheduler, std::move(config.vision.trajectory));
    SPDLOG_INFO(
        "bullet_speed={:.3f}m/s",
        world.get_res<core::trajectory::bullet_speed_data>()->bullet_speed);

    // Register AT Legacy detection pipeline systems
    L2::register_detection_systems(scheduler);
    L2::ldm::register_ldm_systems(
        scheduler, std::move(config.vision.ldm), setup_result.value().camera_config);
    rune::register_rune_detection_systems(scheduler, std::move(config.vision.rune_detector));

    // Register L3 Tracker pipeline (Measurement -> TrackerOutput)
    L3::register_tracker_systems(scheduler, std::move(config.vision.l3->tracker));
    L3::ldm::register_naive_ldm_systems(scheduler, config.vision.naive_ldm);
    energy_meter::register_energy_meter_systems(scheduler, std::move(config.vision.energy_meter));

    // Register L4 Planning pipeline (Aimer + Armor Plan Adapter)
    L4::register_l4_planning_systems(scheduler, std::move(config.vision.l4.get()));

    // Register L5 MPC Weapon system (consumes ControlIntent from L4)
    L5::register_enhanced_weapon_system(scheduler, std::move(config.vision.l5.get()));

    // Register Chiral data collection system (publishes TrackerOutput to shared memory)
    if (chiral) {
        chiral::register_chiral_collector_system(scheduler);
        SPDLOG_INFO("chiral system registered");
    }

    auto quanta_cfg                  = config.vision.quanta;
    quanta_cfg.filter                = config.vision.quanta_filter;
    const auto& stream_camera_config = setup_result.value().camera_config;
    auto quanta_stream_result        = runtime::register_quanta_stream_systems(
        world, scheduler, quanta_cfg, static_cast<int>(stream_camera_config.width),
        static_cast<int>(stream_camera_config.height));
    if (!quanta_stream_result) {
        return std::unexpected(
            fmt::format("register quanta stream: {}", quanta_stream_result.error()));
    }

    runtime::register_runtime_capturer_system(scheduler, config.capturer, config.launch);

    // Build scheduler (freezes resource structure)
    if (auto build_result = scheduler.build(); !build_result) {
        return std::unexpected(fmt::format("build scheduler: {}", build_result.error()));
    }

    return {};
}

} // namespace fcs
