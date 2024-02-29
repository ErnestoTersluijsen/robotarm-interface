//
// Created by ernesto on 29-2-24.
//

#ifndef ROBOTARM_HLD_ROSINTERFACE_HPP
#define ROBOTARM_HLD_ROSINTERFACE_HPP

#include "robotarm_hld/RobotarmHLD.hpp"
#include "robotarm_hld/action/servo.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class RosInterface : public rclcpp::Node
{
  public:
	RosInterface(const std::string& port_name);

  private:
	rclcpp_action::Server<robotarm_hld::action::Servo>::SharedPtr servo_server;

	RobotarmHLD hld;
};

#endif // ROBOTARM_HLD_ROSINTERFACE_HPP
