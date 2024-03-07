//
// Created by ernesto on 27-2-24.
//

#ifndef ROBOTARM_LLD_ROBOTARM_LLD_HPP
#define ROBOTARM_LLD_ROBOTARM_LLD_HPP

#include <boost/asio.hpp>

#include <iostream>
#include <string>

class RobotarmLLD
{
  public:
	/**
	 * @brief Constructor for the Low Level Driver for the robotarm
	 *
	 * @param port_name Name of the port serial is connected to
	 */
	RobotarmLLD(const std::string& port_name);

	/**
	 * @brief Destructor for Low Level Driver
	 *
	 */
	~RobotarmLLD();

	/**
	 * @brief Function that writes the string to the connected serial port
	 *
	 * @param command
	 */
	void write_to_serial(std::string command);

	/**
	 * @brief Converts input into a valid command that the robotarm can understand
	 *
	 * @param servo_id Servo ID of the servo being moved
	 * @param angle Angle that the servo should turn to
	 * @param time Duration that the servo has to move
	 * @return std::string Returns the command that the robotarm can understand
	 */
	std::string input_to_command(uint16_t servo_id, int16_t angle, uint16_t time);

	/**
	 * @brief Function that stops the robotarm moving
	 *
	 */
	void emergency_stop();

  private:
	/**
	 * @brief Sets the serial communication up.
	 *
	 */
	void setup_robotarm();

	/**
	 * @brief Converts a given angle to a PWM value
	 * 
	 * @param id Servo ID
	 * @param angle Angle that has to be converted
	 * @return long PWM
	 */
	long angle_to_pwm(uint16_t id, int16_t angle);

	/**
	 * @brief Limits the max and min angle for safety of the robotarm
	 * 
	 * @param id Servo ID
	 * @param angle Angle where servo has to be turned to
	 * @return long The limited value
	 */
	long limit_angles(uint16_t id, int16_t angle);

	/**
	 * @brief Map function for mapping angle to pwm
	 * 
	 * @param x input
	 * @param in_min input minimum value
	 * @param in_max input maximum value
	 * @param out_min output minimum value
	 * @param out_max output maximum value
	 * @return long output
	 */
	long map(long x, long in_min, long in_max, long out_min, long out_max);

	boost::asio::io_service ioservice;
	boost::asio::serial_port serial;
	boost::asio::streambuf string_stream_buffer;
	std::ostream os;

	/**
	 * @brief Keeps track of how many servo's there are
	 * 
	 */
	unsigned short amount_of_channels;
};

#endif // ROBOTARM_LLD_ROBOTARM_LLD_HPP
