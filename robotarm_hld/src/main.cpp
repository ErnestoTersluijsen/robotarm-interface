#include <robotarm_hld/RosInterface.hpp>

#include <rclcpp/rclcpp.hpp>

#include <iostream>
#include <string>

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RosInterface>("/dev/ttyUSB0"));
	rclcpp::shutdown();

	return 0;
}
