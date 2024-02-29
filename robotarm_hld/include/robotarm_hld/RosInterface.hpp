//
// Created by ernesto on 29-2-24.
//

#ifndef ROBOTARM_HLD_ROSINTERFACE_HPP
#define ROBOTARM_HLD_ROSINTERFACE_HPP

#include "robotarm_hld/RobotarmHLD.hpp"
#include "robotarm_hld/action/servo_positions.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class RosInterface : public rclcpp::Node
{
  public:
	RosInterface(const std::string& port_name);

  private:
	rclcpp_action::GoalResponse handle_servo_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const robotarm_hld::action::ServoPositions::Goal> goal);

	rclcpp_action::CancelResponse handle_servo_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robotarm_hld::action::ServoPositions>> goal_handle);

	void handle_servo_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robotarm_hld::action::ServoPositions>> goal_handle);

	void servo_execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robotarm_hld::action::ServoPositions>> goal_handle);

	rclcpp_action::Server<robotarm_hld::action::ServoPositions>::SharedPtr servo_position_server;

	RobotarmHLD hld;
};

#endif // ROBOTARM_HLD_ROSINTERFACE_HPP
