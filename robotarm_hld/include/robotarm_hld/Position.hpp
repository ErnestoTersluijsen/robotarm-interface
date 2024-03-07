#ifndef ROBOTARM_INTERFACE_POSITION_HPP
#define ROBOTARM_INTERFACE_POSITION_HPP

#include <rclcpp/rclcpp.hpp>

#include <vector>

class Position
{
  public:
	/**
	 * @brief Constructer for a preset position
	 *
	 * @param angles angles that the servo's should be at
	 */
	Position(std::vector<int16_t> angles);

	/**
	 * @brief Function that returns the angles the robotarm should turn to
	 *
	 * @return std::vector<int16_t> Vector with the angles
	 */
	std::vector<int16_t> get_servo_angles() const;

  private:
	/**
	 * @brief Vector containing the angles for this specific preset position
	 *
	 */
	std::vector<int16_t> servo_angles;
};

#endif // ROBOTARM_INTERFACE_POSITION_HPP