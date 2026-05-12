#include "mavsdk_command_dispatcher.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>

using namespace mavsdk;
using arch_nav::constants::CommandResponse;
using arch_nav::constants::ReferenceFrame;

namespace arch_nav_mavsdk {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kYawToleranceRad = 3.0 * kPi / 180.0;
constexpr double kBodyYawStepRad = 80.0 * kPi / 180.0;  // Force deterministic direction.
constexpr std::chrono::seconds kYawOperationTimeout{45};

double normalize_angle_0_2pi(double angle_rad) {
  angle_rad = std::fmod(angle_rad, 2.0 * kPi);
  if (angle_rad < 0.0) angle_rad += 2.0 * kPi;
  return angle_rad;
}

double shortest_angular_error(double target_rad, double current_rad) {
  return std::atan2(
      std::sin(target_rad - current_rad),
      std::cos(target_rad - current_rad));
}

}  // namespace

MavsdkCommandDispatcher::MavsdkCommandDispatcher(
    std::shared_ptr<System> system,
    const MavsdkConfig& config)
    : action_(std::make_unique<Action>(system)),
      mavlink_passthrough_(std::make_unique<MavlinkPassthrough>(system)),
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
  if (landed_state_handle_) {
    telemetry_->unsubscribe_landed_state(*landed_state_handle_);
    landed_state_handle_.reset();
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
        break;
      }
    }

    // Phase 1: release driver resources
    clear_subscriptions();
    resources_released_ = true;

    // Phase 2: notify external consumer
    if (!stop_requested_) {
      std::function<void()> cb;
      {
        std::lock_guard<std::mutex> lock(complete_mutex_);
        cb = std::move(on_complete_);
      }
      if (cb) cb();
    }
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
  land_in_progress_ = true;
  land_completion_notified_ = false;
  land_on_ground_detected_ = false;

  monitor_thread_ = std::thread([this] {
    wait_for_landed_and_notify();
  });

  return CommandResponse::ACCEPTED;
}

CommandResponse MavsdkCommandDispatcher::execute_change_yaw(
    double new_yaw,
    ReferenceFrame frame,
    std::function<void()> on_complete) {
  stop();

  // Heading can be temporarily unavailable right after mode transitions.
  // Retry briefly before rejecting the command.
  double current_heading_rad = std::numeric_limits<double>::quiet_NaN();
  for (int i = 0; i < 30; ++i) {
    const auto heading = telemetry_->heading();
    if (std::isfinite(heading.heading_deg)) {
      current_heading_rad = heading.heading_deg * kPi / 180.0;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  if (!std::isfinite(current_heading_rad)) {
    std::cerr << "[arch_nav_mavsdk] change_yaw denied: heading is not finite"
              << std::endl;
    return CommandResponse::DENIED;
  }

  std::vector<double> target_headings_rad;

  switch (frame) {
    case ReferenceFrame::LOCAL_NED:
      target_headings_rad.push_back(normalize_angle_0_2pi(new_yaw));
      break;
    case ReferenceFrame::LOCAL_ENU:
      target_headings_rad.push_back(normalize_angle_0_2pi(kPi / 2.0 - new_yaw));
      break;
    case ReferenceFrame::BODY_FCS:
      {
        double remaining = new_yaw;
        double accumulated = 0.0;
        while (std::fabs(remaining) > 1e-6) {
          const double step = std::copysign(
              std::min(std::fabs(remaining), kBodyYawStepRad), remaining);
          accumulated += step;
          remaining -= step;
          target_headings_rad.push_back(
              normalize_angle_0_2pi(current_heading_rad + accumulated));
        }
      }
      break;
    default:
      return CommandResponse::DENIED;
  }
  if (target_headings_rad.empty()) {
    target_headings_rad.push_back(current_heading_rad);
  }

  const auto send_goto_to_heading = [this](double heading_rad) {
    const auto pos = telemetry_->position();
    if (!std::isfinite(pos.latitude_deg) ||
        !std::isfinite(pos.longitude_deg) ||
        !std::isfinite(pos.absolute_altitude_m)) {
      std::cerr << "[arch_nav_mavsdk] change_yaw denied: invalid position" << std::endl;
      return false;
    }
    const float target_yaw_deg = static_cast<float>(heading_rad * 180.0 / kPi);
    const auto goto_result = action_->goto_location(
        pos.latitude_deg,
        pos.longitude_deg,
        pos.absolute_altitude_m,
        target_yaw_deg);
    if (goto_result != Action::Result::Success) {
      std::cerr << "[arch_nav_mavsdk] change_yaw denied: goto_location failed with result="
                << goto_result << std::endl;
      return false;
    }
    return true;
  };
  if (!send_goto_to_heading(target_headings_rad.front())) {
    return CommandResponse::DENIED;
  }

  {
    std::lock_guard<std::mutex> lock(complete_mutex_);
    on_complete_ = std::move(on_complete);
  }
  stop_requested_ = false;

  monitor_thread_ = std::thread(
      [this,
       target_headings_rad] {
    // Prevent immediate callback re-entrancy during task start.
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::size_t target_index = 0;
    const auto started_at = std::chrono::steady_clock::now();
    while (!stop_requested_) {
      if (std::chrono::steady_clock::now() - started_at > kYawOperationTimeout) {
        std::cerr << "[arch_nav_mavsdk] change_yaw timeout after "
                  << kYawOperationTimeout.count() << "s" << std::endl;
        break;
      }

      const auto heading = telemetry_->heading();
      if (std::isfinite(heading.heading_deg)) {
        const double current_heading_rad =
            heading.heading_deg * kPi / 180.0;

        const double error = std::fabs(shortest_angular_error(
            target_headings_rad[target_index], current_heading_rad));
        if (error <= kYawToleranceRad) {
          if (target_index + 1 < target_headings_rad.size()) {
            ++target_index;
            const auto pos = telemetry_->position();
            if (std::isfinite(pos.latitude_deg) &&
                std::isfinite(pos.longitude_deg) &&
                std::isfinite(pos.absolute_altitude_m)) {
              const float next_target_yaw_deg = static_cast<float>(
                  target_headings_rad[target_index] * 180.0 / kPi);
              const auto goto_result = action_->goto_location(
                  pos.latitude_deg,
                  pos.longitude_deg,
                  pos.absolute_altitude_m,
                  next_target_yaw_deg);
              if (goto_result != Action::Result::Success) {
                std::cerr << "[arch_nav_mavsdk] change_yaw warning: next step goto failed with result="
                          << goto_result << std::endl;
                break;
              }
            } else {
              std::cerr << "[arch_nav_mavsdk] change_yaw warning: invalid position on step advance"
                        << std::endl;
              break;
            }
          } else {
            // Phase 1: release driver resources
            clear_subscriptions();
            resources_released_ = true;

            // Phase 2: notify external consumer
            if (!stop_requested_) {
              std::function<void()> cb;
              {
                std::lock_guard<std::mutex> lock(complete_mutex_);
                cb = std::move(on_complete_);
              }
              if (cb) cb();
            }
            return;
          }
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    clear_subscriptions();
    resources_released_ = true;
  });

  return CommandResponse::ACCEPTED;
}

void MavsdkCommandDispatcher::complete_landing_if_pending() {
  if (!land_in_progress_.load() ||
      land_completion_notified_.load() ||
      !land_on_ground_detected_.load()) {
    return;
  }

  std::function<void()> cb;
  {
    std::lock_guard<std::mutex> lock(complete_mutex_);
    if (!land_in_progress_.load() ||
        land_completion_notified_.load() ||
        !land_on_ground_detected_.load()) {
      return;
    }

    // Phase 1: release driver resources
    land_completion_notified_ = true;
    land_in_progress_ = false;
    resources_released_ = true;

    // Phase 2: prepare external notification
    cb = std::move(on_complete_);
  }
  if (cb) cb();
}

void MavsdkCommandDispatcher::notify_landing_complete_if_pending() {
  complete_landing_if_pending();
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
        // Phase 1: release driver resources
        clear_subscriptions();
        resources_released_ = true;

        // Phase 2: notify external consumer
        if (!stop_requested_) {
          std::function<void()> cb;
          {
            std::lock_guard<std::mutex> lock(complete_mutex_);
            cb = std::move(on_complete_);
          }
          if (cb) cb();
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
    const bool is_monitor_thread =
        monitor_thread_.get_id() == std::this_thread::get_id();
    if (is_monitor_thread || resources_released_.load())
      monitor_thread_.detach();
    else
      monitor_thread_.join();
  }

  resources_released_ = false;
  std::lock_guard<std::mutex> lock(complete_mutex_);
  on_complete_ = nullptr;
  land_in_progress_ = false;
  land_completion_notified_ = false;
  land_on_ground_detected_ = false;
}

void MavsdkCommandDispatcher::wait_for_landed_and_notify() {
  landed_state_handle_ = telemetry_->subscribe_landed_state(
      [this](Telemetry::LandedState landed_state) {
        if (landed_state == Telemetry::LandedState::OnGround) {
          land_on_ground_detected_ = true;
        }
      });

  while (!stop_requested_) {
    if (land_on_ground_detected_.load()) {
      complete_landing_if_pending();
      clear_subscriptions();
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  clear_subscriptions();
}

}  // namespace arch_nav_mavsdk
