//
// Created by ernesto on 27-2-24.
//

#include "robotarm_lld/RobotarmLLD.hpp"

#include <boost/asio.hpp>

RobotarmLLD::RobotarmLLD(const std::string& port_name) : serial(ioservice, port_name), os(&string_stream_buffer)
{
	setup_robotarm();
	write_to_serial("test");
}

void RobotarmLLD::setup_robotarm()
{
	serial.set_option(boost::asio::serial_port_base::baud_rate(115200));
	serial.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
	serial.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
	serial.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
	serial.set_option(boost::asio::serial_port::character_size(boost::asio::serial_port::character_size(8)));
}

void RobotarmLLD::write_to_serial(std::string command)
{
	os << command;
	boost::asio::write(serial, string_stream_buffer);
}

std::string RobotarmLLD::input_to_command(uint16_t servo_id, int16_t angle, uint16_t time)
{
	std::stringstream ss;
	// ss << '#' << servo_id << 'P' << map_values(servo_id, angle) << 'T' << time << '\r';
	return ss.str();
}
