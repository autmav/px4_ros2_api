#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include "px4_ros_com/msg/target_frd.hpp"

#include <chrono>
#include <iostream>

#include <cmath>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class Control : public rclcpp::Node
{
public:
	explicit Control() : Node("control")
	{
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);
		
		target_relative_frd_subscription_ = this->create_subscription<px4_ros_com::msg::TargetFRD>("target", 10,
		[&](const px4_ros_com::msg::TargetFRD::UniquePtr msg){
			
			xrb = msg->x;
			yrb = msg->y;
			zrb = msg->z;

			if (pow(xrb, 2) + pow(yrb, 2) + pow(zrb, 2) < 0.2){
				std::cout << "mission accomplished "<<"\n";
				return;
			}

		//	std::cout << "target relative position in body frame " << std::endl;
		//	std::cout << "xrb = " << xrb << std::endl;
		//	std::cout << "yrb = " << yrb << std::endl;
		//	std::cout << "zrb = " << zrb << "\n\n";
		});

		vehicle_attitude_subscription_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>("/fmu/out/vehicle_attitude", qos,
		[&](const px4_msgs::msg::VehicleAttitude::UniquePtr msg){
			
			q0 = msg->q[0];
			q1 = msg->q[1];
			q2 = msg->q[2];
			q3 = msg->q[3];

			//calculate FRD to NED transformation matrix elements
			T[0][0] = q0*q0 + q1*q1 - q2*q2 - q3*q3;
			T[1][0] = 2*(q1*q2 + q0*q3);
			T[2][0] = 2*(q1*q3 - q0*q2);
			T[0][1] = 2*(q1*q2 - q0*q3);
			T[1][1] = q0*q0 - q1*q1 + q2*q2 - q3*q3;
			T[2][1] = 2*(q2*q3 + q0*q1);
			T[0][2] = 2*(q1*q3 + q0*q2);
			T[1][2] = 2*(q2*q3 - q0*q1);
			T[2][2] = q0*q0 - q1*q1 - q2*q2 + q3*q3;

			newpitch = asin(-T[2][0]);
			sinnewyaw = T[1][0]/cos(newpitch);
			cosnewyaw = T[0][0]/cos(newpitch);
			if (sinnewyaw >= 0)
				newyaw = acos(cosnewyaw);
			else
				newyaw = 2*3.1416 - acos(cosnewyaw);
		//	std::cout << "yaw: " << 180*newyaw/3.1416 << "\n";
		});

		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		vehicle_attitude_setpoint_publisher_ = this->create_publisher<VehicleAttitudeSetpoint>("/fmu/in/vehicle_attitude_setpoint", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
		timer_ = this->create_wall_timer(100ms, std::bind(&Control::flight_mode_timer_callback, this));
	}

	void arm();
	void disarm();

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<VehicleAttitudeSetpoint>::SharedPtr vehicle_attitude_setpoint_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Subscription<px4_ros_com::msg::TargetFRD>::SharedPtr target_relative_frd_subscription_;
	rclcpp::Subscription<VehicleAttitude>::SharedPtr vehicle_attitude_subscription_;
	
	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	std::atomic<float> q0, q1, q2, q3, xrb, yrb, zrb, xrl, yrl, zrl, x, y, z, vx, vy, vz, roll, pitch, yaw, newyaw, newpitch, sinnewyaw, cosnewyaw;
	std::atomic<bool> offposition, offvelocity, offattitude, offbodyrate;
	std::atomic<float> T[3][3];
	int state = 0;
	int stateOld = 0;
	uint64_t offboard_setpoint_counter_ = 0;

	void publish_offboard_control_mode(bool offposition = 0, bool offvelocity = 1, bool offattitude = 0, bool offbodyrate = 0);
	void publish_vehicle_attitude_setpoint(float roll = 0, float pitch = 0, float yaw = 0);
	void publish_trajectory_setpoint(float x = 0, float y = 0, float z = 0, float vx = 0, float vy = 0, float vz = 0, float yaw = 0);
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param3 = 0.0);
	void flight_mode_timer_callback();
};

void Control::flight_mode_timer_callback()
{
	if(offboard_setpoint_counter_ == 1){
		offposition = false;
		offvelocity = true;
		offattitude = false;
		offbodyrate = false;
		x = std::numeric_limits<float>::quiet_NaN();
		y = std::numeric_limits<float>::quiet_NaN();
		z = std::numeric_limits<float>::quiet_NaN();
		roll = std::numeric_limits<float>::quiet_NaN();
		pitch = std::numeric_limits<float>::quiet_NaN();
		this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 4, 2);
		this->arm();
		}
	
	xrl = T[0][0]*xrb + T[0][1]*yrb + T[0][2]*zrb;
	yrl = T[1][0]*xrb + T[1][1]*yrb + T[1][2]*zrb;
	zrl = T[2][0]*xrb + T[2][1]*yrb + T[2][2]*zrb;
	vx = 3*xrl/sqrt(pow(xrl, 2) + pow(yrl, 2));
	vy = 3*yrl/sqrt(pow(xrl, 2) + pow(xrl, 2));
	vz = 5*zrl;
	yaw = newyaw + atan(yrb/xrb);
					
	if(offboard_setpoint_counter_ == 80)
		this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
		
	publish_offboard_control_mode(offposition, offvelocity, offattitude, offbodyrate);
	publish_trajectory_setpoint(x, y, z, vx, vy, vz, yaw);
	publish_vehicle_attitude_setpoint(roll, pitch, yaw);
		
	offboard_setpoint_counter_++;			
}

void Control::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void Control::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void Control::publish_offboard_control_mode(bool offposition, bool offvelocity, bool offattitude, bool offbodyrate)
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

void Control::publish_trajectory_setpoint(float x, float y, float z, float vx, float vy, float vz, float yaw)
{
	TrajectorySetpoint msg{};
	msg.position = {x, y, z};
	msg.velocity = {vx, vy, vz};
	msg.yaw = yaw;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

void Control::publish_vehicle_attitude_setpoint(float roll, float pitch, float yaw)
{
	VehicleAttitudeSetpoint msg{};
	msg.yaw_sp_move_rate = std::numeric_limits<float>::quiet_NaN();
	msg.q_d[0] = cos(yaw/2)*cos(pitch/2)*cos(roll/2)+sin(yaw/2)*sin(pitch/2)*sin(roll/2);
	msg.q_d[1] = cos(yaw/2)*cos(pitch/2)*sin(roll/2)-sin(yaw/2)*sin(pitch/2)*cos(roll/2);
	msg.q_d[2] = cos(yaw/2)*sin(pitch/2)*cos(roll/2)+sin(yaw/2)*cos(pitch/2)*sin(roll/2);
	msg.q_d[3] = sin(yaw/2)*cos(pitch/2)*cos(roll/2)-cos(yaw/2)*sin(pitch/2)*sin(roll/2);
	msg.thrust_body = {0, 0, -0.71};
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_attitude_setpoint_publisher_->publish(msg);
}


/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void Control::publish_vehicle_command(uint16_t command, float param1, float param2, float param3)
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
	rclcpp::spin(std::make_shared<Control>());
	rclcpp::shutdown();
	return 0;
}