//
// Created by ernesto on 29-2-24.
//

#include "robotarm_hld/RosInterface.hpp"

#include <rclcpp/rclcpp.hpp>

RosInterface::RosInterface(const std::string& port_name) : Node("interface"), hld(port_name)
{
	std::cout << "RosInterface" << std::endl;
	servo_position_server = rclcpp_action::create_server<robotarm_hld::action::ServoPositions>(this, "servo", std::bind(&RosInterface::handle_servo_goal, this, std::placeholders::_1, std::placeholders::_2), std::bind(&RosInterface::handle_servo_cancel, this, std::placeholders::_1), std::bind(&RosInterface::handle_servo_accepted, this, std::placeholders::_1));
}

rclcpp_action::GoalResponse RosInterface::handle_servo_goal([[maybe_unused]] const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const robotarm_hld::action::ServoPositions::Goal> goal)
{
	if (goal->servoids.size() == 0 || goal->servoids.size() != goal->angles.size())
	{
		return rclcpp_action::GoalResponse::REJECT;
	}
	return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RosInterface::handle_servo_cancel([[maybe_unused]] const std::shared_ptr<rclcpp_action::ServerGoalHandle<robotarm_hld::action::ServoPositions>> goal_handle)
{
	return rclcpp_action::CancelResponse::ACCEPT;
}

void RosInterface::handle_servo_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robotarm_hld::action::ServoPositions>> goal_handle)
{
	std::thread{std::bind(&RosInterface::servo_execute, this, std::placeholders::_1), goal_handle}.detach();
}

void RosInterface::servo_execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robotarm_hld::action::ServoPositions>> goal_handle)
{
	auto goal = goal_handle->get_goal();

	std::cout << "servo ID:";
	for (auto servoid : goal->servoids)
	{
		std::cout << " " << servoid;
	}
	std::cout << std::endl;


	std::cout << "angles:";
	for (auto angle : goal->angles)
	{
		std::cout << " " << angle;
	}
	std::cout << std::endl;

	std::cout << "speed: " << goal->speed << std::endl;

	auto result = std::make_shared<robotarm_hld::action::ServoPositions::Result>();
	result->response = "succeeded";
	goal_handle->succeed(result);
}
