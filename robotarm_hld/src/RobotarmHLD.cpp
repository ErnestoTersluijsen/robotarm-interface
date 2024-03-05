//
// Created by ernesto on 29-2-24.
//

#include "robotarm_hld/RobotarmHLD.hpp"

#include "robotarm_hld/Position.hpp"

RobotarmHLD::RobotarmHLD(const std::string& port_name) : lld(port_name)
{
	std::cout << "RobotarmHLD" << std::endl;
	presets.push_back(Position({0, 60, 108, 45, 0, 0}));	// PARK
	presets.push_back(Position({0, 48, 74, 10, 0, 0}));		// READY
	presets.push_back(Position({0, 30, 14, 0, 0, 0}));		// STRAIGHT UP

}

void RobotarmHLD::move_servos(std::vector<uint16_t> servo_ids, std::vector<int16_t> joint_angles, uint16_t speed)
{
	for (size_t i = 0; i < servo_ids.size(); ++i)
	{
		lld.write_to_serial(lld.input_to_command(servo_ids.at(i), joint_angles.at(i), speed));
	}
}

void RobotarmHLD::move_to_preset(int preset, uint16_t speed)
{
	std::vector<uint16_t> servo_ids;
	std::vector<int16_t> preset_angle = presets.at(preset).get_servo_angles();

	for (size_t i = 0; i < preset_angle.size(); ++i)
	{
		servo_ids.push_back(static_cast<uint16_t>(i));
	}

	move_servos(servo_ids, preset_angle, speed);
}

void RobotarmHLD::emergency_stop()
{
	lld.emergency_stop();
}