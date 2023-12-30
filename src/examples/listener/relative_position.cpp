#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

//#include "interfaces/msg/target_frd.hpp"
//#include <std_msgs/msg/float32_multi_array.hpp>

//#include <eigen3/Eigen/Eigen>
//#include <eigen3/Eigen/Geometry>
//#include <array>

#include <chrono>
#include <iostream>

#include "px4_ros_com/msg/target_frd.hpp"

//#include "../../../include/px4_ros_com/frame_transforms.h"


using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

//using namespace px4_ros_com;
//using namespace frame_transforms;
//using namespace utils;
//using namespace quaternion;

class RelativePosition : public rclcpp::Node
{
public:
	explicit RelativePosition() : Node("relative_position")
	{

		//struct attitude
		//{ std::atomic<float> x, y, z, q; }attitude;

		//std::atomic<float> x;
		//std::atomic<float> y;
		//std::atomic<float> z;
		//std::atomic<float> q;
		
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);

		vehicle_attitude_subscription_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>("/fmu/out/vehicle_attitude", qos,
		[&](const px4_msgs::msg::VehicleAttitude::UniquePtr msg){
			
			q0 = msg->q[0];
			q1 = msg->q[1];
			q2 = msg->q[2];
			q3 = msg->q[3];

			//calculate NED to FRD transformation matrix elements
			T[0][0] = q0*q0 + q1*q1 - q2*q2 - q3*q3;
			T[0][1] = 2*(q1*q2 + q0*q3);
			T[0][2] = 2*(q1*q3 - q0*q2);
			T[1][0] = 2*(q1*q2 - q0*q3);
			T[1][1] = q0*q0 - q1*q1 + q2*q2 - q3*q3;
			T[1][2] = 2*(q2*q3 + q0*q1);
			T[2][0] = 2*(q1*q3 + q0*q2);
			T[2][1] = 2*(q2*q3 - q0*q1);
			T[2][2] = q0*q0 - q1*q1 - q2*q2 + q3*q3;

		//print transformation matrix
		//	std::cout<<"Transformation matrix:"<<std::endl;
		//	for(int i = 0; i < 3; i++){
		//		for(int j = 0; j < 3; j++){
		//			std::cout<<T[i][j]<<" ";
		//		}
		//		std::cout<<"\n";
		//	}
		//	std::cout<<"\n\n";
			//std::cout << "size of rotation matrix " << (msg->q).toRotationMatrix().size() << std::endl;
			//std::cout << "size of quaternion " << (msg->q).size() << std::endl;
			//std::cout << "roll: " << (quaternion_to_euler((msg->q))).x() << std::endl;
			//std::cout << "pitch: " << (quaternion_to_euler(msg->q)).y() << std::endl;
			//std::cout << "yaw: " << (quaternion_to_euler(msg->q)).z() << std::endl;
			//std::cout << "T: " << (msg->q).toRotationMatrix().transpose << std::endl;
		//	std::cout << "q[0]: " << msg->q[0] << std::endl;
		//	std::cout << "q[1]: " << msg->q[1] << std::endl;
		//	std::cout << "q[2]: " << msg->q[2] << std::endl;
		//	std::cout << "q[3]: " << msg->q[3] << "\n\n";
			//x = msg->q[0];
			//y = msg->q[1];
			//z = msg->q[2];
			//q = msg->q[3];msg->q[0]
		});

		vehicle_local_position_subscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position", qos,
		[&](const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg){
			
			x = msg->x;
			y = msg->y;
			z = msg->z;
			
			xr = 5-x;
			yr = 5-y;
			zr = -5-z;

			xrb = T[0][0]*xr + T[0][1]*yr + T[0][2]*zr;
			yrb = T[1][0]*xr + T[1][1]*yr + T[1][2]*zr;
			zrb = T[2][0]*xr + T[2][1]*yr + T[2][2]*zr;
			std::cout << "target relative position in body frame " << std::endl;
			std::cout << "x = " << x << std::endl;
			std::cout << "y = " << y << std::endl;
			std::cout << "z = " << z << "\n\n";
		});
		
		target_relative_frd_publisher_ = this->create_publisher<px4_ros_com::msg::TargetFRD>("target", 10);
		timer_ = this->create_wall_timer(50ms, std::bind(&RelativePosition::timer_callback, this));
	}

private:
	void timer_callback()
    {
      auto message = px4_ros_com::msg::TargetFRD();
      message.x = xrb;
      message.y = yrb;
      message.z = zrb;
    //  RCLCPP_INFO(this->get_logger(), "Publishing: '%f', '%f', '%f'", message.x, message.y, message.z);
      target_relative_frd_publisher_->publish(message);
    }
	
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Subscription<VehicleAttitude>::SharedPtr vehicle_attitude_subscription_;
	rclcpp::Subscription<VehicleLocalPosition>::SharedPtr vehicle_local_position_subscription_;
	rclcpp::Publisher<px4_ros_com::msg::TargetFRD>::SharedPtr target_relative_frd_publisher_;
//	rclcpp::Publisher<std::array<float, 3>>::SharedPtr target_relative_frd_publisher_;

	
	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped
	std::atomic<float> q0, q1, q2, q3, x, y, z, xr, yr, zr, xrb, yrb, zrb;
	std::atomic<float> T[3][3];
};

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RelativePosition>());

	rclcpp::shutdown();
	return 0;
}
