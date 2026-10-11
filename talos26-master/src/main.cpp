#include "fcs/runtime/boot.hpp"
#include "fcs_visualization/foxglove_sink.hpp"
#include "fcs_visualization/foxglove_systems.hpp"
#include "runtime/build_info.hpp"
#include "runtime/config_loader.hpp"
#include "scheduler/error_formatter.hpp"
#include "scheduler/scheduler.hpp"
#include "spdlog_hook.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <thread>
#include <variant>

namespace {

std::atomic<bool> g_shutdown_requested{false};

void signal_handler(int) { g_shutdown_requested.store(true, std::memory_order_release); }

} // namespace

#ifndef TALOS_DISABLE_PROGRAM_MAIN
int main() {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    init_logger();
    hook_cstream();
    const auto start = std::chrono::system_clock::now();
    const auto build = fcs::build_info();
    SPDLOG_INFO(
        "build version={}@{} git={} host={}", build.git_branch, build.build_date, build.git_commit,
        build.build_host);

    auto config = fcs::load_config("at_vision.toml");
    if (!config) {
        SPDLOG_CRITICAL("{}", config.error());
        return 1;
    }

    // Extract foxglove config before boot() consumes the rest
    // (foxglove setup requires fcs_visualization, which is not linked by the fcs library)
    auto foxglove_cfg = config->foxglove;

    talos::Scheduler scheduler(config->scheduler);

    // Foxglove visualization setup (separate library dependency)
    if (foxglove_cfg.enabled) {
        auto server_result = fcs::visualization::create_foxglove_server(foxglove_cfg);
        if (server_result) {
            std::shared_ptr<fcs::visualization::FoxgloveServer> server = std::move(*server_result);
            fcs::visualization::attach_foxglove_sink(*server);
            scheduler.world().insert_resource(server);
            fcs::visualization::register_foxglove_systems(
                std::holds_alternative<fcs::hardware::DaedalusConfig>(config->backend), scheduler,
                &scheduler);
            if (foxglove_cfg.transport == fcs::FoxgloveTransport::WebSocket) {
                SPDLOG_INFO(
                    "Foxglove WebSocket enabled on ws://{}:{}", foxglove_cfg.host,
                    foxglove_cfg.port);
            } else {
                SPDLOG_INFO("Foxglove MCAP enabled at {}", foxglove_cfg.mcap_path);
            }
        } else {
            SPDLOG_WARN(
                "Foxglove transport failed to initialize: {}. Visualization disabled",
                server_result.error());
        }
    } else {
        SPDLOG_INFO("Foxglove visualization disabled by config");
    }

    if (auto r = fcs::boot(scheduler, std::move(config.value())); !r) {
        SPDLOG_CRITICAL("{}", r.error());
        return 1;
    }

    SPDLOG_INFO(
        "init done. ({}ms)", std::chrono::duration_cast<std::chrono::milliseconds>(
                                 std::chrono::system_clock::now() - start)
                                 .count());

    std::atomic<bool> scheduler_finished{false};
    std::thread shutdown_watcher([&scheduler, &scheduler_finished]() {
        bool shutdown_logged = false;
        while (!scheduler_finished.load(std::memory_order_acquire)) {
            if (g_shutdown_requested.load(std::memory_order_acquire)) {
                if (!shutdown_logged) {
                    SPDLOG_INFO("shutdown signal received, stopping scheduler...");
                    shutdown_logged = true;
                }
                scheduler.stop();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    });

    const auto result = scheduler.run();
    scheduler_finished.store(true, std::memory_order_release);
    if (shutdown_watcher.joinable()) {
        shutdown_watcher.join();
    }

    if (!result) {
        SPDLOG_CRITICAL("run scheduler: {}", result.error());
    }
}
#endif
