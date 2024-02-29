#include <rclcpp/rclcpp.hpp>

#include <robotarm_lld/RobotarmLLD.hpp>

int main(int argc, char** argv)
{
	std::string bruh = "balls";
	rclcpp::init(argc, argv);
	// rclcpp::spin(std::make_shared<>());
	rclcpp::shutdown();
}
