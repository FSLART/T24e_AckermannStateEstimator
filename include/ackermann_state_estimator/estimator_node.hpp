#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"
#include <chrono>

#define WHEEL_RADIUS 0.175
#define WHEEL_BASE 1.58
#define TRACK 1.22

// acceptable percentage of wheel speed ratio to be considered slipping
#define SLIP_THRESHOLD 0.8

using namespace std::chrono_literals;

struct wheel_speed {
	long long timestamp;
	double expected;
	double measured;
	double ratio;
	bool set = false;
};

class EstimatorNode : public rclcpp::Node {
	public:
		EstimatorNode();

	private:
		geometry_msgs::msg::Twist cur_twist;
		ackermann_msgs::msg::AckermannDrive cur_ackermann;
		bool ackermann_received = false;

		// [0][x] -> left wheels
		// [1][x] -> right wheels
		struct wheel_speed wheel_speeds[2][2];

		// ackermann subscriber method
		void ackermann_callback(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg);

		// wheel speeds subscriber methods
		void left_front_wheel_callback(const std_msgs::msg::Float64::SharedPtr msg);
		void right_front_wheel_callback(const std_msgs::msg::Float64::SharedPtr msg);
		void left_back_wheel_callback(const std_msgs::msg::Float64::SharedPtr msg);
		void right_back_wheel_callback(const std_msgs::msg::Float64::SharedPtr msg);

		void get_cur_expected_wheel_speeds();
		void get_cur_wheel_speeds_ratio();

		bool are_all_wheel_speeds_set();

		rclcpp::TimerBase::SharedPtr timer_;

		// ackermann subscriber
		rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr ackermann_subscriber_;

		// wheel speeds subscriber
		rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr left_front_wheel_subscriber_;
		rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr right_front_wheel_subscriber_;
		rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr left_back_wheel_subscriber_;
		rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr right_back_wheel_subscriber_;

		// twist publisher
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};