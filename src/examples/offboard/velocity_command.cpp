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

	std::atomic<float> x, y, z, vx, vy, vz, yaw;
	std::atomic<bool> offposition, offvelocity, offattitude, offbodyrate;
	int state = 0;
	int stateOld = 0;

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	
	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_ = 0;

	void publish_offboard_control_mode(bool offposition = 0, bool offvelocity = 1, bool offattitude = 0, bool offbodyrate = 0);
	void publish_trajectory_setpoint(float x = 0, float y = 0, float z = 0, float vx = 0, float vy = 0, float vz = 0, float yaw = 0);
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param3 = 0.0);
	void flight_mode_timer_callback();
};

void OffboardControl::flight_mode_timer_callback()
{
			//switching between states
			switch(state)
			{
				case 0:
					
					if(offboard_setpoint_counter_ == 140)
						state = 10;
					break;

				case 10:
					
					if(offboard_setpoint_counter_ == 25)
						state = 20;
					break;

				case 20:
					if(offboard_setpoint_counter_ == 25)
						state = 30;
					break;

				case 30:
					if(offboard_setpoint_counter_ == 25)
						state = 40;
					break;

				case 40:
					if(offboard_setpoint_counter_ == 25)
						state = 50;
					break;

				case 50:
					
					if(offboard_setpoint_counter_ == 25)
						return;
					break;	
			}
			
			//states behaviour			
			switch(state)
			{
				case 0:
					
					if(offboard_setpoint_counter_ == 1){
						offposition = true;
						offvelocity = false;
						offattitude = false;
						offbodyrate = false;
						x = 0;
						y = 0;
						z = -3;
						vx = std::numeric_limits<float>::quiet_NaN();
						vy = std::numeric_limits<float>::quiet_NaN();
						vz = std::numeric_limits<float>::quiet_NaN();
						yaw = 3.14/2;
					}
					
					if(offboard_setpoint_counter_ == 20){
						this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
						this->arm();
					}
											
					break;

				case 10:
					// offboard mode _ start with const velocity & then yaw (after 5 seconds)
					if(offboard_setpoint_counter_ == 1){
						offposition = false;
						offvelocity = true;
						x = std::numeric_limits<float>::quiet_NaN();
						y = std::numeric_limits<float>::quiet_NaN();
						z = std::numeric_limits<float>::quiet_NaN();
						vx = 0;
						vy = 2;
						vz = 0;
					}
					break;
								
				case 20:
					// offboard mode _ start with const velocity & then yaw (after 5 seconds)
					if(offboard_setpoint_counter_ == 1){
						vx = 2;
						vy = 0;
					}
					break;

				case 30:
					// offboard mode _ start with const velocity & then yaw (after 5 seconds)
					if(offboard_setpoint_counter_ == 1){
						vx = 0;
						vy = -2;
					}
					break;

				case 40:
					// offboard mode _ start with const velocity & then yaw (after 5 seconds)
					if(offboard_setpoint_counter_ == 1){
						vx = -2;
						vy = 0;
					}
					break;	

				case 50:
					// offboard mode _ start with const velocity & then yaw (after 5 seconds)
					if(offboard_setpoint_counter_ == 1)
						vx = 0;
					break;				
			}

			publish_offboard_control_mode(offposition, offvelocity, offattitude, offbodyrate);
			publish_trajectory_setpoint(x, y, z, vx, vy, vz, yaw);
		//	publish_vehicle_attitude_setpoint(roll, pitch, yaw);
		//	std::cout << "offboard_setpoint_counter_ = " << offboard_setpoint_counter_ << std::endl;

			if(state != stateOld)
				{
					offboard_setpoint_counter_ = 0;
					stateOld = state;
				}
			offboard_setpoint_counter_++;			
}

void OffboardControl::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode(bool offposition, bool offvelocity, bool offattitude, bool offbodyrate)
{
	OffboardControlMode msg{};
	msg.position = offposition;
	msg.velocity = offvelocity;
	msg.acceleration = false;
	msg.attitude = offattitude;
	msg.body_rate = offbodyrate;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */

void OffboardControl::publish_trajectory_setpoint(float x, float y, float z, float vx, float vy, float vz, float yaw)
{
	TrajectorySetpoint msg{};
	msg.position = {x, y, z};
	msg.velocity = {vx, vy, vz};
	msg.yaw = yaw;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
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