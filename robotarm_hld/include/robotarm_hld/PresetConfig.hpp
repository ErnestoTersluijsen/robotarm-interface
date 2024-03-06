#ifndef ROBOTARM_HLD_PRESETCONFIG_HPP
#define ROBOTARM_HLD_PRESETCONFIG_HPP

#include "robotarm_hld/Position.hpp"

#include <rclcpp/rclcpp.hpp>

#include <vector>

enum presetPosition
{
	PARK = 0,
	READY = 1,
	STRAIGHT_UP = 2
};

namespace preset_config
{
	std::vector<Position> presets =
	{
		Position({0, 60, 108, 45, 0, 0}),
		Position({0, 48, 74, 10, 0, 0}),
		Position({0, 30, 14, 0, 0, 0})
	};
} // namespace preset_config

#endif // ROBOTARM_HLD_PRESETCONFIG_HPP
