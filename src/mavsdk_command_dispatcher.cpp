#include "mavsdk_command_dispatcher.hpp"

#include <chrono>

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
  param_->set_param_int("MIS_TKO_LAND_REQ", 0);
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
  if (result != Action::Result::Success) return CommandResponse::DENIED;
  return CommandResponse::ACCEPTED;
}

CommandResponse MavsdkCommandDispatcher::execute_disarm() {
  auto result = action_->disarm();
  if (result != Action::Result::Success) return CommandResponse::DENIED;
  return CommandResponse::ACCEPTED;
}

CommandResponse MavsdkCommandDispatcher::execute_takeoff(
    double height, ReferenceFrame frame,
    std::function<void()> on_complete,
    arch_nav::report::TakeoffDriverOperationData& driver_data) {
  if (frame != ReferenceFrame::LOCAL_NED) return CommandResponse::DENIED;

  stop();

  auto set_alt_result = action_->set_takeoff_altitude(static_cast<float>(height));
  if (set_alt_result != Action::Result::Success) return CommandResponse::DENIED;

  auto takeoff_result = action_->takeoff();
  if (takeoff_result != Action::Result::Success) return CommandResponse::DENIED;

  {
    std::lock_guard<std::mutex> lock(complete_mutex_);
    on_complete_ = std::move(on_complete);
  }
  stop_requested_ = false;

  monitor_thread_ = std::thread([this, &driver_data] {
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
      auto pos = telemetry_->position();
      driver_data.current_altitude.store(
          static_cast<double>(pos.relative_altitude_m));

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      if (completed->load()) {
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
  if (result != Action::Result::Success) return CommandResponse::DENIED;

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
    std::function<void()> on_complete,
    arch_nav::report::WaypointDriverOperationData& driver_data) {
  if (frame != ReferenceFrame::GLOBAL_WGS84) return CommandResponse::DENIED;

  stop();

  Mission::MissionPlan plan;
  for (const auto& wp : waypoints) {
    Mission::MissionItem item{};
    item.latitude_deg        = wp.lat;
    item.longitude_deg       = wp.lon;
    item.relative_altitude_m = static_cast<float>(wp.alt);
    item.speed_m_s           = config_.default_speed_m_s;
    item.is_fly_through      = true;
    plan.mission_items.push_back(item);
  }

  if (!plan.mission_items.empty()) {
    plan.mission_items.back().is_fly_through = false;
  }

  auto upload_result = mission_->upload_mission(plan);
  if (upload_result != Mission::Result::Success) return CommandResponse::DENIED;

  std::this_thread::sleep_for(std::chrono::milliseconds(config_.mission_upload_delay_ms));

  auto start_result = mission_->start_mission();
  if (start_result != Mission::Result::Success) return CommandResponse::DENIED;

  {
    std::lock_guard<std::mutex> lock(complete_mutex_);
    on_complete_ = std::move(on_complete);
  }
  stop_requested_ = false;

  monitor_thread_ = std::thread([this, &driver_data] {
    auto progress_updated = std::make_shared<std::atomic<bool>>(false);
    auto last_current = std::make_shared<std::atomic<int>>(0);
    auto last_total = std::make_shared<std::atomic<int>>(0);

    mission_progress_handle_ = mission_->subscribe_mission_progress(
        [progress_updated, last_current, last_total](Mission::MissionProgress progress) {
          last_current->store(progress.current);
          last_total->store(progress.total);
          progress_updated->store(true);
        });

    while (!stop_requested_) {
      if (progress_updated->exchange(false)) {
        driver_data.current_waypoint.store(last_current->load());
        driver_data.total_waypoints.store(last_total->load());
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      auto finished = mission_->is_mission_finished();
      if (finished.first == Mission::Result::Success && finished.second) {
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
    std::function<void()> /*on_complete*/) {
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
