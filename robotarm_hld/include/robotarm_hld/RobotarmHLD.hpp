//
// Created by ernesto on 29-2-24.
//

#ifndef ROBOTARM_HLD_ROBOTARMHLD_HPP
#define ROBOTARM_HLD_ROBOTARMHLD_HPP

#include "robotarm_hld/Position.hpp"
#include "robotarm_lld/RobotarmLLD.hpp"

class RobotarmHLD
{
  public:
	/**
	 * @brief Contructor for Robotarm High Level Driver
	 *
	 * @param port_name port name where robotarm is connected
	 */
	RobotarmHLD(const std::string& port_name);

	/**
	 * @brief Function that moves the servo's of the robotarm to a specific angle and speed
	 *
	 * @param servo_ids Vector with the servo IDs you want to move
	 * @param joint_angles Angle where the servo should move too
	 * @param speed Duration for the robotarm to move
	 */
	void move_servos(std::vector<uint16_t> servo_ids, std::vector<int16_t> joint_angles, uint16_t speed);

	/**
	 * @brief Moves the robotarm to a specific preset
	 *
	 * @param preset ID of the preset, can be found in PresetConfig.hpp
	 * @param speed
	 */
	void move_to_preset(uint16_t preset, uint16_t speed);

	/**
	 * @brief Function that stops the robotarm and blocks all new requests
	 *
	 */
	void emergency_stop();

	/**
	 * @brief Getter for getting the amount of presets there are
	 *
	 * @return long Returns amount of presets
	 */
	long get_amount_of_presets() const;

  private:
	/**
	 * @brief Function that initialises serial communication
	 *
	 */
	void initialise_robotarm();

	/**
	 * @brief Low Level Driver object
	 * 
	 */
	RobotarmLLD lld;
};

#endif // ROBOTARM_HLD_ROBOTARMHLD_HPP
