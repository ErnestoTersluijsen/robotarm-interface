//
// Created by ernesto on 29-2-24.
//

#include "robotarm_hld/RobotarmHLD.hpp"

RobotarmHLD::RobotarmHLD(const std::string& port_name) : lld(port_name)
{
	std::cout << "RobotarmHLD" << std::endl;
}

void RobotarmHLD::move_servos(std::vector<uint16_t> servo_ids, std::vector<int16_t> joint_angles, uint16_t speed)
{
}