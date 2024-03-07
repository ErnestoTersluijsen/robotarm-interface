//
// Created by ernesto on 27-2-24.
//

#include "robotarm_lld/RobotarmLLD.hpp"

#include "robotarm_lld/config.hpp"

#include <boost/asio.hpp>

RobotarmLLD::RobotarmLLD(const std::string& port_name) : serial(ioservice, port_name), os(&string_stream_buffer), amount_of_channels(6)
{
	setup_robotarm();
}

RobotarmLLD::~RobotarmLLD()
{
	serial.close();
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
	ss << '#' << servo_id << 'P' << angle_to_pwm(servo_id, angle) << 'T' << time << '\r' << '\n';
	return ss.str();
}

void RobotarmLLD::emergency_stop()
{
	for (size_t i = 0; i < amount_of_channels; ++i)
	{
		std::stringstream ss;
		ss << "STOP " << i << "\r";
		write_to_serial(ss.str());
	}
}

long RobotarmLLD::angle_to_pwm(uint16_t id, int16_t angle)
{
	return map(limit_angles(id, angle), config::servo_limits.at(id).first, config::servo_limits.at(id).second, 500, 2500);
}

long RobotarmLLD::limit_angles(uint16_t id, int16_t angle)
{
	if (id == 3)
	{
		if (config::servo_limits.at(id).first < angle)
		{
			return config::servo_limits.at(id).first;
		}
		else if (config::servo_limits.at(id).second > angle)
		{
			return config::servo_limits.at(id).second;
		}
		else
		{
			return angle;
		}
	}
	else
	{
		if (config::servo_limits.at(id).first > angle)
		{
			return config::servo_limits.at(id).first;
		}
		else if (config::servo_limits.at(id).second < angle)
		{
			return config::servo_limits.at(id).second;
		}
		else
		{
			return angle;
		}
	}
}

long RobotarmLLD::map(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}