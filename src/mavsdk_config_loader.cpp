#include "mavsdk_config_loader.hpp"

#include <yaml-cpp/yaml.h>

namespace arch_nav_mavsdk {

MavsdkConfig MavsdkConfigLoader::load_from(const std::string& path) {
  MavsdkConfig config;
  YAML::Node yaml = YAML::LoadFile(path);

  if (yaml["connection_url"])
    config.connection_url = yaml["connection_url"].as<std::string>();
  if (yaml["discover_timeout_s"])
    config.discover_timeout_s = yaml["discover_timeout_s"].as<double>();
  if (yaml["default_altitude_m"])
    config.default_altitude_m = yaml["default_altitude_m"].as<float>();
  if (yaml["default_speed_m_s"])
    config.default_speed_m_s = yaml["default_speed_m_s"].as<float>();

  return config;
}

}  // namespace arch_nav_mavsdk
