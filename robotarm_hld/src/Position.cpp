#include "robotarm_hld/Position.hpp"

Position::Position(uint16_t servo_base_angle, uint16_t servo_shoulder_angle, uint16_t servo_elbow_angle, uint16_t servo_wrist_angle, uint16_t servo_gripper_angle, uint16_t servo_wrist_rotate)
{
	servo_angles = {servo_base_angle, servo_shoulder_angle, servo_elbow_angle, servo_wrist_angle, servo_gripper_angle, servo_wrist_rotate};
}

// void Position::commands_to_serial(LowLevelDriver& lld, uint16_t time)
// {
// 	for (uint16_t i = 1; i < servo_angles.size(); ++i)
// 	{
// 		lld.write_to_serial(lld.input_to_command(i, servo_angles.at(i), time));
// 	}
// }