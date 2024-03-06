#include "robotarm_hld/Position.hpp"

Position::Position(std::vector<int16_t> angles) : servo_angles(angles)
{
}

std::vector<int16_t> Position::get_servo_angles() const
{
	return servo_angles;
}