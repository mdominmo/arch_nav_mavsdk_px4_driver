#ifndef ARCH_NAV_MAVSDK__DISPATCHERS__MAVSDK_COMMAND_DISPATCHER_HPP_
#define ARCH_NAV_MAVSDK__DISPATCHERS__MAVSDK_COMMAND_DISPATCHER_HPP_

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/param/param.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include "dispatchers/i_command_dispatcher.hpp"
#include "config/mavsdk_config.hpp"

namespace arch_nav_mavsdk {

class MavsdkCommandDispatcher : public arch_nav::dispatchers::ICommandDispatcher {
 public:
  explicit MavsdkCommandDispatcher(
      std::shared_ptr<mavsdk::System> system,
      const MavsdkConfig& config);

  ~MavsdkCommandDispatcher() override;

  void execute_takeoff(double height, std::function<void()> on_complete) override;
  void execute_land(std::function<void()> on_complete) override;
  void execute_waypoint_following(
      std::vector<arch_nav::vehicle::GeoWaypoint> waypoints,
      std::function<void()> on_complete) override;
  void execute_trajectory(
      std::vector<arch_nav::vehicle::TrajectoryPoint> trajectory,
      std::function<void()> on_complete) override;
  void execute_arm() override;
  void execute_disarm() override;
  void stop() override;

 private:
  void wait_for_landed_and_notify();
  void clear_subscriptions();

  std::unique_ptr<mavsdk::Action>    action_;
  std::unique_ptr<mavsdk::Mission>   mission_;
  std::unique_ptr<mavsdk::Param>     param_;
  std::unique_ptr<mavsdk::Telemetry> telemetry_;
  MavsdkConfig config_;

  std::atomic<bool>      stop_requested_{false};
  std::function<void()>  on_complete_;
  std::mutex             complete_mutex_;
  std::thread            monitor_thread_;

  std::optional<mavsdk::Telemetry::FlightModeHandle>      flight_mode_handle_;
  std::optional<mavsdk::Mission::MissionProgressHandle>    mission_progress_handle_;
};

}  // namespace arch_nav_mavsdk

#endif  // ARCH_NAV_MAVSDK__DISPATCHERS__MAVSDK_COMMAND_DISPATCHER_HPP_
