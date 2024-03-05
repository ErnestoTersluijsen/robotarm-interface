#include "robotarm_hld/Position.hpp"

Position::Position(std::vector<int16_t> angles) : servo_angles(angles)
{
}

// std::string Position::commands_to_serial(uint16_t time)
// {
// 	for (uint16_t i = 1; i < servo_angles.size(); ++i)
// 	{
// 		lld.write_to_serial(lld.input_to_command(i, servo_angles.at(i), time));
// 	}
// }

std::vector<int16_t> Position::get_servo_angles() const
{
	return servo_angles;
}