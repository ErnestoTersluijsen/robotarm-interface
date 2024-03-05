//
// Created by ernesto on 29-2-24.
//

#ifndef ROBOTARM_HLD_ROSINTERFACE_HPP
#define ROBOTARM_HLD_ROSINTERFACE_HPP

#include "robotarm_hld/RobotarmHLD.hpp"
#include "robotarm_hld/action/position_preset.hpp"
#include "robotarm_hld/action/servo_positions.hpp"
#include "robotarm_hld/srv/emergency_stop.hpp"

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

	rclcpp_action::GoalResponse handle_position_preset_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const robotarm_hld::action::PositionPreset::Goal> goal);

	rclcpp_action::CancelResponse handle_position_preset_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robotarm_hld::action::PositionPreset>> goal_handle);

	void handle_position_preset_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robotarm_hld::action::PositionPreset>> goal_handle);

	void position_preset_execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robotarm_hld::action::PositionPreset>> goal_handle);

	void handle_emergency_stop_request(const std::shared_ptr<robotarm_hld::srv::EmergencyStop::Request> request, std::shared_ptr<robotarm_hld::srv::EmergencyStop::Response> response);

	rclcpp_action::Server<robotarm_hld::action::ServoPositions>::SharedPtr servo_position_server;

	rclcpp_action::Server<robotarm_hld::action::PositionPreset>::SharedPtr position_preset_server;

	rclcpp::Service<robotarm_hld::srv::EmergencyStop>::SharedPtr emergency_stop_service;

	RobotarmHLD hld;

	bool emergency_stop;
};

#endif // ROBOTARM_HLD_ROSINTERFACE_HPP
