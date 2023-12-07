#include <px4_msgs/msg/vehicle_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>


using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl() : Node("offboard_control")
	{
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
		this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 4, 2);		
		this->arm();		
	}

	void arm();

private:
	rclcpp::TimerBase::SharedPtr timer_;
	
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	
	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped


	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param3 = 0.0);
};
/**
 * @brief Send a command to Arm the vehicle
 */

void OffboardControl::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2, float param3)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.param3 = param3;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());
	rclcpp::shutdown();
	return 0;
}
