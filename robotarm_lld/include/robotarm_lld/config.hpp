#ifndef ROBOTARM_LLD_CONFIG_HPP
#define ROBOTARM_LLD_CONFIG_HPP

#include <rclcpp/rclcpp.hpp>

#include <utility>

namespace config
{
	/**
	 * @brief Vector for keeping track of the turning limits imposed on the servo's
	 * 
	 */
	std::vector<std::pair<int16_t, int16_t>> servo_limits =
		{
			{-90, 90},
			{-30, 90},
			{0, 135},
			{90, -90},
			{-90, 90},
			{-90, 90}
		};
} // namespace config

#endif // ROBOTARM_LLD_CONFIG_HPP