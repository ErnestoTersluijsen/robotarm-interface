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
	RobotarmLLD(const std::string& port_name);

	~RobotarmLLD();

	void write_to_serial(std::string command);

	std::string input_to_command(uint16_t servo_id, int16_t angle, uint16_t time);

	void emergency_stop();

  private:
	void setup_robotarm();

	long map(long x, long in_min, long in_max, long out_min, long out_max);

	long limit_angles(uint16_t id, int16_t angle);

	boost::asio::io_service ioservice;
	boost::asio::serial_port serial;
	boost::asio::streambuf string_stream_buffer;
	std::ostream os;

	unsigned short amount_of_channels;
};

#endif // ROBOTARM_LLD_ROBOTARM_LLD_HPP
