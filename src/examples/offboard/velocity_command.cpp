#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

#include "rmw/types.h"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl() : Node("offboard_control")
	{
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

		timer_ = this->create_wall_timer(100ms, std::bind(&OffboardControl::flight_mode_timer_callback, this));
	}

	void arm();
	void disarm();

	int state = 0;
	int stateOld = 0;

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	
	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_ = 0;   

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param3 = 0.0);
	void flight_mode_timer_callback();
};

void OffboardControl::flight_mode_timer_callback()
{
	if(offboard_setpoint_counter_ == 10){
		// auto takeoff mode
		this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 4, 2);
						
		// Arm the vehicle
		this->arm();
	}
		// offboard mode start with const velocity
	if(offboard_setpoint_counter_ == 150)
		this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

	publish_offboard_control_mode();
	publish_trajectory_setpoint();

	offboard_setpoint_counter_++;			
}

void OffboardControl::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

void OffboardControl::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = false;
	msg.velocity = true;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

void OffboardControl::publish_trajectory_setpoint()
{
	TrajectorySetpoint msg{};
	msg.position = {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN()};
	msg.velocity = {2, 0, 0};
	msg.yaw = 3.14/2;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
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
