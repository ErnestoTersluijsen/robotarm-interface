#ifndef ROBOTARM_INTERFACE_POSITION_HPP
#define ROBOTARM_INTERFACE_POSITION_HPP

#include <rclcpp/rclcpp.hpp>

#include <vector>

class Position
{
  public:
	Position(std::vector<int16_t> angles);

	std::vector<int16_t> get_servo_angles() const;

  private:
	std::vector<int16_t> servo_angles;
};

#endif // ROBOTARM_INTERFACE_POSITION_HPP