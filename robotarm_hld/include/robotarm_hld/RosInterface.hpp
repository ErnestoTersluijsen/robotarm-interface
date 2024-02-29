//
// Created by ernesto on 29-2-24.
//

#ifndef ROBOTARM_HLD_ROSINTERFACE_HPP
#define ROBOTARM_HLD_ROSINTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class RosInterface
{
  public:
	RosInterface();

  private:
	// rclcpp_action::Server<int>::SharedPtr position_server;
};

#endif // ROBOTARM_HLD_ROSINTERFACE_HPP
