//
// Created by ernesto on 29-2-24.
//

#include "robotarm_hld/RobotarmHLD.hpp"

#include "robotarm_hld/Position.hpp"
#include "robotarm_hld/PresetConfig.hpp"

RobotarmHLD::RobotarmHLD(const std::string& port_name) : lld(port_name)
{
	initialise_robotarm();
}

void RobotarmHLD::move_servos(std::vector<uint16_t> servo_ids, std::vector<int16_t> joint_angles, uint16_t speed)
{
	for (size_t i = 0; i < servo_ids.size(); ++i)
	{
		lld.write_to_serial(lld.input_to_command(servo_ids.at(i), joint_angles.at(i), speed));
	}
}

void RobotarmHLD::move_to_preset(uint16_t preset, uint16_t speed)
{
	std::vector<uint16_t> servo_ids;
	std::vector<int16_t> preset_angle = preset_config::presets.at(preset).get_servo_angles();

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

long RobotarmHLD::get_amount_of_presets() const
{
	return preset_config::presets.size();
}

void RobotarmHLD::initialise_robotarm()
{
	move_to_preset(PARK, 2200);
}