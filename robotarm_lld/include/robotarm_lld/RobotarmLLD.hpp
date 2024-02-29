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
	RobotarmLLD(std::string port_name);

	void write_to_serial(std::string command);

  private:
	void setup_robotarm();

	boost::asio::serial_port serial;
	boost::asio::io_service ioservice;
	boost::asio::streambuf string_stream_buffer;
	std::ostream os;
};

#endif // ROBOTARM_LLD_ROBOTARM_LLD_HPP
