#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>

using namespace std::chrono_literals;

class EstimatorNode : public rclcpp::Node {
	public:
		EstimatorNode();

	private:
		void odom_callback();
		void ackermann_callback(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg) const;
		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr ackermann_subscriber_;
		// TODO: wheel speeds subscriber
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};