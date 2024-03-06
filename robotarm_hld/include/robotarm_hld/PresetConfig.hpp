#ifndef ROBOTARM_HLD_PRESETCONFIG_HPP
#define ROBOTARM_HLD_PRESETCONFIG_HPP

#include "robotarm_hld/Position.hpp"

#include <rclcpp/rclcpp.hpp>

#include <vector>

namespace preset_config
{
// presets[0] = PARK, presets[0] = READY, presets[0] = STRAIGHT UP
std::vector<Position> presets = {Position({0, 60, 108, 45, 0, 0}), Position({0, 48, 74, 10, 0, 0}), Position({0, 30, 14, 0, 0, 0})};

} // namespace preset_config

#endif // ROBOTARM_HLD_PRESETCONFIG_HPP
