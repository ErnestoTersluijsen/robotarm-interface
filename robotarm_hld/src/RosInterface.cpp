//
// Created by ernesto on 29-2-24.
//

#include "robotarm_hld/RosInterface.hpp"

#include <rclcpp/rclcpp.hpp>

RosInterface::RosInterface(const std::string& port_name) : Node("interface"), hld(port_name), emergency_stop(false)
{
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
	auto feedback = std::make_shared<robotarm_hld::action::ServoPositions::Feedback>();
	auto result = std::make_shared<robotarm_hld::action::ServoPositions::Result>();

	hld.move_servos(goal->servo_ids, goal->joint_angles, goal->speed);

	auto start_time = std::chrono::high_resolution_clock::now();

	while (rclcpp::ok())
	{
		if (goal_handle->is_canceling())
		{
			result->response = "cancelled";
			goal_handle->succeed(result);
			return;
		}

		if (emergency_stop)
		{
			result->response = "emergency stop";
			goal_handle->succeed(result);
			return;
		}

		auto current_time = std::chrono::high_resolution_clock::now();
		auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count();

		feedback->progress = static_cast<uint16_t>((static_cast<double>(elapsed_time) / static_cast<double>(goal->speed)) * 100);
		goal_handle->publish_feedback(feedback);

		if (elapsed_time >= goal->speed)
		{
			result->response = "succeeded";
			goal_handle->succeed(result);
			return;
		}
	}
}

// ================================================================================================

rclcpp_action::GoalResponse RosInterface::handle_position_preset_goal([[maybe_unused]] const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const robotarm_hld::action::PositionPreset::Goal> goal)
{
	if (emergency_stop || goal->position >= hld.get_amount_of_presets())
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
	auto feedback = std::make_shared<robotarm_hld::action::PositionPreset::Feedback>();
	auto result = std::make_shared<robotarm_hld::action::PositionPreset::Result>();

	auto goal = goal_handle->get_goal();

	hld.move_to_preset(goal->position, goal->speed);

	auto start_time = std::chrono::high_resolution_clock::now();

	while (rclcpp::ok())
	{
		if (goal_handle->is_canceling())
		{
			result->response = "cancelled";
			goal_handle->succeed(result);
			return;
		}

		if (emergency_stop)
		{
			result->response = "emergency stop";
			goal_handle->succeed(result);
			return;
		}

		auto current_time = std::chrono::high_resolution_clock::now();
		auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count();

		feedback->progress = static_cast<uint16_t>((static_cast<double>(elapsed_time) / static_cast<double>(goal->speed)) * 100);
		goal_handle->publish_feedback(feedback);

		if (elapsed_time >= goal->speed)
		{
			result->response = "succeeded";
			goal_handle->succeed(result);
			return;
		}
	}
}

// ================================================================================================

void RosInterface::handle_emergency_stop_request([[maybe_unused]] const std::shared_ptr<robotarm_hld::srv::EmergencyStop::Request> request, std::shared_ptr<robotarm_hld::srv::EmergencyStop::Response> response)
{
	hld.emergency_stop();

	emergency_stop = true;

	response->response = "succeeded";
}
