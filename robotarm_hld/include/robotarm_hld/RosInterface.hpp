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
	/**
	 * @brief Defaultconstructor for the ROS communication
	 *
	 * @param port_name Name of the port where serial is connected
	 */
	RosInterface(const std::string& port_name);

  private:
	/**
	 * @brief Action server handle goal function for servo position
	 *
	 */
	rclcpp_action::GoalResponse handle_servo_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const robotarm_hld::action::ServoPositions::Goal> goal);

	/**
	 * @brief Action server handle cancel function for servo position
	 *
	 */
	rclcpp_action::CancelResponse handle_servo_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robotarm_hld::action::ServoPositions>> goal_handle);

	/**
	 * @brief Action server handle accepted function for servo position
	 *
	 */
	void handle_servo_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robotarm_hld::action::ServoPositions>> goal_handle);

	/**
	 * @brief Action server execute function for servo position. Has to be run on a seperate thread
	 *
	 */
	void servo_execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robotarm_hld::action::ServoPositions>> goal_handle);

	/**
	 * @brief Action server handle goal function for position preset
	 *
	 */
	rclcpp_action::GoalResponse handle_position_preset_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const robotarm_hld::action::PositionPreset::Goal> goal);

	/**
	 * @brief Action server handle cancel function for position preset
	 *
	 */
	rclcpp_action::CancelResponse handle_position_preset_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robotarm_hld::action::PositionPreset>> goal_handle);

	/**
	 * @brief Action server handle accepted function for position preset
	 *
	 */
	void handle_position_preset_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robotarm_hld::action::PositionPreset>> goal_handle);

	/**
	 * @brief Action server execute function for position preset. Has to be run on a seperate thread
	 *
	 */
	void position_preset_execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robotarm_hld::action::PositionPreset>> goal_handle);

	/**
	 * @brief Service for emergency stop
	 *
	 */
	void handle_emergency_stop_request(const std::shared_ptr<robotarm_hld::srv::EmergencyStop::Request> request, std::shared_ptr<robotarm_hld::srv::EmergencyStop::Response> response);

	/**
	 * @brief Function for printing the current state to ROS INFO
	 *
	 * @param state Current state
	 */
	void print_state_info(std::string state);

	/**
	 * @brief Function for printing the current event to ROS DEBUG
	 *
	 * @param event Current event
	 */
	void print_event_info(std::string event);

	/**
	 * @brief Servo position server object.
	 *
	 */
	rclcpp_action::Server<robotarm_hld::action::ServoPositions>::SharedPtr servo_position_server;

	/**
	 * @brief Position preset server object.
	 *
	 */
	rclcpp_action::Server<robotarm_hld::action::PositionPreset>::SharedPtr position_preset_server;

	/**
	 * @brief Emergency stop service
	 *
	 */
	rclcpp::Service<robotarm_hld::srv::EmergencyStop>::SharedPtr emergency_stop_service;

	/**
	 * @brief High Level Driver object
	 *
	 */
	RobotarmHLD hld;

	/**
	 * @brief Boolean that is true when emergency stop has been triggered
	 *
	 */
	bool emergency_stop;
};

#endif // ROBOTARM_HLD_ROSINTERFACE_HPP
