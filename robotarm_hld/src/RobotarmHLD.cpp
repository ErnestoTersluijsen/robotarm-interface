//
// Created by ernesto on 29-2-24.
//

#include "robotarm_hld/RobotarmHLD.hpp"

RobotarmHLD::RobotarmHLD(const std::string& port_name) : lld(port_name)
{
	std::cout << "RobotarmHLD" << std::endl;
}