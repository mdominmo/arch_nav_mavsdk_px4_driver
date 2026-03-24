#include "dispatchers/mavsdk_command_dispatcher.hpp"

#include <chrono>
#include <iostream>

using namespace mavsdk;
using arch_nav::constants::CommandResponse;
using arch_nav::constants::ReferenceFrame;

namespace arch_nav_mavsdk {

MavsdkCommandDispatcher::MavsdkCommandDispatcher(
    std::shared_ptr<System> system,
    const MavsdkConfig& config)
    : action_(std::make_unique<Action>(system)),
      mission_(std::make_unique<Mission>(system)),
      param_(std::make_unique<Param>(system)),
      telemetry_(std::make_unique<Telemetry>(system)),
      config_(config) {
  // Disable mission feasibility requirement for takeoff/landing items
  // so that missions can be started while already airborne
  auto result = param_->set_param_int("MIS_TKO_LAND_REQ", 0);
  if (result != Param::Result::Success) {
    std::cerr << "[MAVSDK] Warning: could not set MIS_TKO_LAND_REQ=0: "
              << result << std::endl;
  }
}

MavsdkCommandDispatcher::~MavsdkCommandDispatcher() {
  stop();
}

void MavsdkCommandDispatcher::clear_subscriptions() {
  if (flight_mode_handle_) {
    telemetry_->unsubscribe_flight_mode(*flight_mode_handle_);
    flight_mode_handle_.reset();
  }
  if (mission_progress_handle_) {
    mission_->unsubscribe_mission_progress(*mission_progress_handle_);
    mission_progress_handle_.reset();
  }
}

CommandResponse MavsdkCommandDispatcher::execute_arm() {
  auto result = action_->arm();
  if (result != Action::Result::Success) {
    std::cerr << "[MAVSDK] Arm failed: " << result << std::endl;
    return CommandResponse::DENIED;
  }
  return CommandResponse::ACCEPTED;
}

CommandResponse MavsdkCommandDispatcher::execute_disarm() {
  auto result = action_->disarm();
  if (result != Action::Result::Success) {
    std::cerr << "[MAVSDK] Disarm failed: " << result << std::endl;
    return CommandResponse::DENIED;
  }
  return CommandResponse::ACCEPTED;
}

CommandResponse MavsdkCommandDispatcher::execute_takeoff(
    double height, ReferenceFrame /*frame*/,
    std::function<void()> on_complete) {
  stop();

  auto set_alt_result = action_->set_takeoff_altitude(static_cast<float>(height));
  if (set_alt_result != Action::Result::Success) {
    std::cerr << "[MAVSDK] Set takeoff altitude failed: " << set_alt_result << std::endl;
    return CommandResponse::DENIED;
  }

  auto takeoff_result = action_->takeoff();
  if (takeoff_result != Action::Result::Success) {
    std::cerr << "[MAVSDK] Takeoff failed: " << takeoff_result << std::endl;
    return CommandResponse::DENIED;
  }

  std::cout << "[MAVSDK] Takeoff initiated at " << height << "m" << std::endl;

  {
    std::lock_guard<std::mutex> lock(complete_mutex_);
    on_complete_ = std::move(on_complete);
  }
  stop_requested_ = false;

  monitor_thread_ = std::thread([this] {
    auto saw_takeoff = std::make_shared<std::atomic<bool>>(false);
    auto completed = std::make_shared<std::atomic<bool>>(false);

    flight_mode_handle_ = telemetry_->subscribe_flight_mode(
        [saw_takeoff, completed](Telemetry::FlightMode mode) {
          if (mode == Telemetry::FlightMode::Takeoff) {
            saw_takeoff->store(true);
            return;
          }
          if (saw_takeoff->load() && mode == Telemetry::FlightMode::Hold) {
            completed->store(true);
          }
        });

    while (!stop_requested_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      if (completed->load()) {
        std::cout << "[MAVSDK] Takeoff complete" << std::endl;
        std::lock_guard<std::mutex> lock(complete_mutex_);
        if (on_complete_) {
          on_complete_();
          on_complete_ = nullptr;
        }
        break;
      }
    }
    clear_subscriptions();
  });

  return CommandResponse::ACCEPTED;
}

CommandResponse MavsdkCommandDispatcher::execute_land(
    std::function<void()> on_complete) {
  stop();

  auto result = action_->land();
  if (result != Action::Result::Success) {
    std::cerr << "[MAVSDK] Land failed: " << result << std::endl;
    return CommandResponse::DENIED;
  }

  std::cout << "[MAVSDK] Landing initiated" << std::endl;

  {
    std::lock_guard<std::mutex> lock(complete_mutex_);
    on_complete_ = std::move(on_complete);
  }
  stop_requested_ = false;

  monitor_thread_ = std::thread([this] {
    wait_for_landed_and_notify();
  });

  return CommandResponse::ACCEPTED;
}

CommandResponse MavsdkCommandDispatcher::execute_waypoint_following(
    std::vector<arch_nav::vehicle::Waypoint> waypoints,
    ReferenceFrame frame,
    std::function<void()> on_complete) {
  if (frame != ReferenceFrame::GLOBAL_WGS84) {
    std::cerr << "[MAVSDK] Waypoint following only supports GLOBAL_WGS84 frame" << std::endl;
    return CommandResponse::DENIED;
  }

  stop();

  // Build mission plan from Waypoints (using lat/lon/alt union members)
  Mission::MissionPlan plan;
  for (const auto& wp : waypoints) {
    Mission::MissionItem item{};
    item.latitude_deg       = wp.lat;
    item.longitude_deg      = wp.lon;
    item.relative_altitude_m = static_cast<float>(wp.alt);
    item.speed_m_s          = config_.default_speed_m_s;
    item.is_fly_through     = true;
    plan.mission_items.push_back(item);
  }

  // Make last waypoint a stop point
  if (!plan.mission_items.empty()) {
    plan.mission_items.back().is_fly_through = false;
  }

  // Upload mission
  auto upload_result = mission_->upload_mission(plan);
  if (upload_result != Mission::Result::Success) {
    std::cerr << "[MAVSDK] Mission upload failed: " << upload_result << std::endl;
    return CommandResponse::DENIED;
  }
  std::cout << "[MAVSDK] Mission uploaded (" << waypoints.size()
            << " waypoints)" << std::endl;

  // Small delay to let PX4 process the uploaded mission
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // Start mission
  auto start_result = mission_->start_mission();
  if (start_result != Mission::Result::Success) {
    std::cerr << "[MAVSDK] Mission start failed: " << start_result << std::endl;
    return CommandResponse::DENIED;
  }
  std::cout << "[MAVSDK] Mission started" << std::endl;

  {
    std::lock_guard<std::mutex> lock(complete_mutex_);
    on_complete_ = std::move(on_complete);
  }
  stop_requested_ = false;

  // Monitor mission progress
  monitor_thread_ = std::thread([this] {
    mission_progress_handle_ = mission_->subscribe_mission_progress(
        [this](Mission::MissionProgress progress) {
          std::cout << "[MAVSDK] Mission progress: " << progress.current
                    << "/" << progress.total << std::endl;
        });

    while (!stop_requested_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      auto finished = mission_->is_mission_finished();
      if (finished.first == Mission::Result::Success && finished.second) {
        std::cout << "[MAVSDK] Mission complete" << std::endl;
        clear_subscriptions();
        std::lock_guard<std::mutex> lock(complete_mutex_);
        if (on_complete_) {
          on_complete_();
          on_complete_ = nullptr;
        }
        break;
      }
    }
  });

  return CommandResponse::ACCEPTED;
}

CommandResponse MavsdkCommandDispatcher::execute_trajectory(
    std::vector<arch_nav::vehicle::TrajectoryPoint> /*trajectory*/,
    ReferenceFrame /*frame*/,
    std::function<void()> on_complete) {
  std::cerr << "[MAVSDK] execute_trajectory not yet implemented "
            << "(requires Offboard plugin)" << std::endl;
  if (on_complete) on_complete();
  return CommandResponse::NOT_SUPPORTED;
}

void MavsdkCommandDispatcher::stop() {
  stop_requested_ = true;
  clear_subscriptions();

  if (monitor_thread_.joinable()) {
    monitor_thread_.join();
  }

  std::lock_guard<std::mutex> lock(complete_mutex_);
  on_complete_ = nullptr;
}

void MavsdkCommandDispatcher::wait_for_landed_and_notify() {
  while (!stop_requested_) {
    if (!telemetry_->armed()) {
      std::cout << "[MAVSDK] Landing complete (disarmed)" << std::endl;
      std::lock_guard<std::mutex> lock(complete_mutex_);
      if (on_complete_) {
        on_complete_();
        on_complete_ = nullptr;
      }
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
}

}  // namespace arch_nav_mavsdk
