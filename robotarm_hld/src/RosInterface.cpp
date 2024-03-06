//
// Created by ernesto on 29-2-24.
//

#include "robotarm_hld/RosInterface.hpp"

#include <rclcpp/rclcpp.hpp>

RosInterface::RosInterface(const std::string& port_name) : Node("interface"), hld(port_name), emergency_stop(false)
{
	std::cout << "RosInterface" << std::endl;

	servo_position_server = rclcpp_action::create_server<robotarm_hld::action::ServoPositions>(this, "servo", std::bind(&RosInterface::handle_servo_goal, this, std::placeholders::_1, std::placeholders::_2), std::bind(&RosInterface::handle_servo_cancel, this, std::placeholders::_1), std::bind(&RosInterface::handle_servo_accepted, this, std::placeholders::_1));

	position_preset_server = rclcpp_action::create_server<robotarm_hld::action::PositionPreset>(this, "position_preset", std::bind(&RosInterface::handle_position_preset_goal, this, std::placeholders::_1, std::placeholders::_2), std::bind(&RosInterface::handle_position_preset_cancel, this, std::placeholders::_1), std::bind(&RosInterface::handle_position_preset_accepted, this, std::placeholders::_1));

	emergency_stop_service = create_service<robotarm_hld::srv::EmergencyStop>("emergency_stop", std::bind(&RosInterface::handle_emergency_stop_request, this, std::placeholders::_1, std::placeholders::_2));
}

// ================================================================================================

rclcpp_action::GoalResponse RosInterface::handle_servo_goal([[maybe_unused]] const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const robotarm_hld::action::ServoPositions::Goal> goal)
{
	if ((goal->servo_ids.size() == 0) || (goal->servo_ids.size() != goal->joint_angles.size()) || emergency_stop)
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

	hld.move_servos(goal->servo_ids, goal->joint_angles, goal->speed);

	auto result = std::make_shared<robotarm_hld::action::ServoPositions::Result>();
	result->response = "succeeded";
	goal_handle->succeed(result);
}

// ================================================================================================

rclcpp_action::GoalResponse RosInterface::handle_position_preset_goal([[maybe_unused]] const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const robotarm_hld::action::PositionPreset::Goal> goal)
{
	if (emergency_stop) // TODO: ADD CHECK FOR VALID POSITION (amount_of_presets < goal->position)
	{
		return rclcpp_action::GoalResponse::REJECT;
	}
	return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RosInterface::handle_position_preset_cancel([[maybe_unused]] const std::shared_ptr<rclcpp_action::ServerGoalHandle<robotarm_hld::action::PositionPreset>> goal_handle)
{
	return rclcpp_action::CancelResponse::ACCEPT;
}

void RosInterface::handle_position_preset_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robotarm_hld::action::PositionPreset>> goal_handle)
{
	std::thread{std::bind(&RosInterface::position_preset_execute, this, std::placeholders::_1), goal_handle}.detach();
}

void RosInterface::position_preset_execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robotarm_hld::action::PositionPreset>> goal_handle)
{
	auto goal = goal_handle->get_goal();

	hld.move_to_preset(goal->position, goal->speed);

	// auto feedback = std::make_shared<robotarm_hld::action::PositionPreset::Feedback>();
	// feedback->progress = 50;
	// goal_handle->publish_feedback(feedback);

	auto result = std::make_shared<robotarm_hld::action::PositionPreset::Result>();
	result->response = "succeeded";
	goal_handle->succeed(result);
}

// ================================================================================================

void RosInterface::handle_emergency_stop_request([[maybe_unused]] const std::shared_ptr<robotarm_hld::srv::EmergencyStop::Request> request, std::shared_ptr<robotarm_hld::srv::EmergencyStop::Response> response)
{
	hld.emergency_stop();

	emergency_stop = true;

	response->response = "succeeded";
}
