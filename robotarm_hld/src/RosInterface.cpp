//
// Created by ernesto on 29-2-24.
//

#include "robotarm_hld/RosInterface.hpp"

#include <rclcpp/rclcpp.hpp>

RosInterface::RosInterface(const std::string& port_name) : Node("interface") , hld(port_name)
{
	std::cout << "RosInterface" << std::endl;
	// servo_server = rclcpp_action::create_server<robotarm_hld::action::Servo>(this, "servo", );
}
