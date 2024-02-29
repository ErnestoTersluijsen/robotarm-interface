#include <robotarm_hld/RosInterface.hpp>

#include <rclcpp/rclcpp.hpp>

#include <iostream>
#include <string>

int main(int argc, char** argv)
{
	std::cout << "main" << std::endl;
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RosInterface>("/dev/pts/7"));
	rclcpp::shutdown();

	return 0;
}
