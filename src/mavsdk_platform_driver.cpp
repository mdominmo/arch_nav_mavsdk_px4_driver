#include "mavsdk_platform_driver.hpp"

#include <chrono>
#include <iostream>
#include <mutex>
#include <optional>
#include <stdexcept>

#include <mavsdk/plugins/telemetry/telemetry.h>

#include "dispatchers/mavsdk_command_dispatcher.hpp"

using namespace mavsdk;

namespace arch_nav_mavsdk {

struct MavsdkPlatformDriver::Internals {
  MavsdkCommandDispatcher dispatcher;

  Internals(std::shared_ptr<System> system, const MavsdkConfig& config)
      : dispatcher(system, config) {}
};

MavsdkPlatformDriver::MavsdkPlatformDriver(const MavsdkConfig& config)
    : config_(config) {
  Mavsdk::Configuration mavsdk_config{ComponentType::GroundStation};
  mavsdk_ = std::make_unique<Mavsdk>(mavsdk_config);

  auto conn_result = mavsdk_->add_any_connection(config_.connection_url);
  if (conn_result != ConnectionResult::Success) {
    throw std::runtime_error(
        "[MAVSDK] Connection failed: " + std::to_string(static_cast<int>(conn_result)));
  }
  std::cout << "[MAVSDK] Connected to " << config_.connection_url << std::endl;

  auto system_opt = mavsdk_->first_autopilot(config_.discover_timeout_s);
  if (!system_opt) {
    throw std::runtime_error("[MAVSDK] No autopilot found within timeout");
  }
  system_ = system_opt.value();
  std::cout << "[MAVSDK] Autopilot discovered" << std::endl;

  internals_ = std::make_unique<Internals>(system_, config_);
}

MavsdkPlatformDriver::~MavsdkPlatformDriver() {
  stop();
}

arch_nav::dispatchers::ICommandDispatcher& MavsdkPlatformDriver::dispatcher() {
  return internals_->dispatcher;
}

void MavsdkPlatformDriver::start(arch_nav::context::VehicleContext& context) {
  running_ = true;
  telemetry_thread_ = std::thread([this, &context] {
    Telemetry telemetry(system_);

    std::mutex buf_mutex;
    std::optional<arch_nav::vehicle::GlobalPosition> buf_gp;
    std::optional<arch_nav::vehicle::Kinematics> buf_kin;
    std::optional<arch_nav::vehicle::VehicleStatus> buf_status;

    telemetry.subscribe_position(
        [&buf_mutex, &buf_gp](Telemetry::Position pos) {
          std::lock_guard<std::mutex> lock(buf_mutex);
          buf_gp.emplace(
              pos.latitude_deg, pos.longitude_deg,
              static_cast<double>(pos.absolute_altitude_m));
        });

    telemetry.subscribe_velocity_ned(
        [&buf_mutex, &buf_kin, &telemetry](Telemetry::VelocityNed vel) {
          auto pos = telemetry.position();
          auto heading = telemetry.heading();
          std::lock_guard<std::mutex> lock(buf_mutex);
          buf_kin.emplace(
              0.0, 0.0, 0.0,
              vel.north_m_s, vel.east_m_s, vel.down_m_s,
              0.0, 0.0, 0.0,
              pos.latitude_deg, pos.longitude_deg,
              static_cast<double>(pos.absolute_altitude_m),
              heading.heading_deg);
        });

    telemetry.subscribe_armed(
        [&buf_mutex, &buf_status](bool armed) {
          std::lock_guard<std::mutex> lock(buf_mutex);
          buf_status.emplace(
              arch_nav::constants::ControlState::KERNEL_CONTROLLED,
              armed ? arch_nav::constants::ArmState::ARMED
                    : arch_nav::constants::ArmState::DISARMED);
        });

    while (running_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));

      std::optional<arch_nav::vehicle::GlobalPosition> gp;
      std::optional<arch_nav::vehicle::Kinematics> kin;
      std::optional<arch_nav::vehicle::VehicleStatus> status;
      {
        std::lock_guard<std::mutex> lock(buf_mutex);
        gp.swap(buf_gp);
        kin.swap(buf_kin);
        status.swap(buf_status);
      }

      if (gp) context.update(*gp);
      if (kin) context.update(*kin);
      if (status) context.update(*status);
    }

    telemetry.subscribe_position(nullptr);
    telemetry.subscribe_velocity_ned(nullptr);
    telemetry.subscribe_armed(nullptr);
  });

  std::cout << "[MAVSDK] Driver started" << std::endl;
}

void MavsdkPlatformDriver::stop() {
  running_ = false;

  if (internals_) {
    internals_->dispatcher.stop();
  }

  if (telemetry_thread_.joinable()) {
    telemetry_thread_.join();
  }

  internals_.reset();
  system_.reset();
  mavsdk_.reset();

  std::cout << "[MAVSDK] Driver stopped" << std::endl;
}

}  // namespace arch_nav_mavsdk
