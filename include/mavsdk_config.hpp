#ifndef ARCH_NAV_MAVSDK__CONFIG__MAVSDK_CONFIG_HPP_
#define ARCH_NAV_MAVSDK__CONFIG__MAVSDK_CONFIG_HPP_

#include <cstdint>
#include <string>

namespace arch_nav_mavsdk {

struct MavsdkConfig {
  std::string connection_url = "udp://localhost:14581";
  double      discover_timeout_s = 15.0;
  float       default_altitude_m = 10.0f;
  float       default_speed_m_s  = 5.0f;
};

}  // namespace arch_nav_mavsdk

#endif  // ARCH_NAV_MAVSDK__CONFIG__MAVSDK_CONFIG_HPP_
