#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

#define CANDRIVER_SERIAL 1
extern "C" {
#include "libjaguar.h"
#include "can.h"
}
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node {
public:
	MinimalSubscriber()
		: Node("jaguar_twist_drivetrain_node") {
		this->declare_parameter("PORT", "/dev/ttyJAGUAR");
		this->declare_parameter("LEFT_ID", 1);
		this->declare_parameter("RIGHT_ID", 2);
		this->declare_parameter("NOMINAL_VOLTAGE", 12.0f);

		int res = open_can_connection(&conn, this->get_parameter("PORT").as_string().c_str());
		if (res != 0) {
			RCLCPP_ERROR(this->get_logger(), "Could not open CAN connection");
			return;
		}

		subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
			"cmd_vel", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));

		voltcomp_enable(&conn, this->get_parameter("LEFT_ID").as_int());
		voltcomp_enable(&conn, this->get_parameter("RIGHT_ID").as_int());
	}

private:
	void topic_callback(const geometry_msgs::msg::Twist& msg) const {

		float lwheel_vel = std::min(1.0, std::max(-1.0, msg.linear.x + msg.angular.z));
		float rwheel_vel = std::min(1.0, std::max(-1.0, msg.linear.x - msg.angular.z));
		RCLCPP_INFO(this->get_logger(), "Left: %.2f, Right: %.2f\n", lwheel_vel, rwheel_vel);


		float V = this->get_parameter("NOMINAL_VOLTAGE").as_double();
		voltcomp_set((CANConnection*)&this->conn, this->get_parameter("LEFT_ID").as_int(), float_to_fixed16(V * lwheel_vel));
		voltcomp_set((CANConnection*)&this->conn, this->get_parameter("RIGHT_ID").as_int(), float_to_fixed16(V * rwheel_vel));
		sys_heartbeat((CANConnection*)&this->conn, this->get_parameter("RIGHT_ID").as_int());
		sys_heartbeat((CANConnection*)&this->conn, this->get_parameter("LEFT_ID").as_int());
	}
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
	CANConnection conn;
};

int main(int argc, char* argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MinimalSubscriber>());
	rclcpp::shutdown();
	return 0;
}