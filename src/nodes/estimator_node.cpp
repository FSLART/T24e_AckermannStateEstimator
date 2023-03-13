#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/Odometry.hpp"
#include "ackermann_msgs/msg/AckermannDrive.hpp"

class EstimatorNode : public rlcpp::Node {
	public:
		EstimatorNode() : Node("estimator_node"), count_(0) {
			publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
			ackermann_subscriber_ = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(
					"ackermann", 10, std::bind(&EstimatorNode::ackermann_callback, this, _1));
			// TODO: subscribe wheel speeds

			
			timer_ = this->create_wall_timer(500ms, std::bind(&EstimatorNode::odom_callback, this));
		}
	private:
		void odom_callback() {
			// TODO called on each timer tick to publish odom
		}
		void ackermann_callback(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg) const {
			RCLCPP_INFO(this->get_logger(), "New ackermann message");
		}
		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr ackermann_subscriber_;
		// TODO: wheel speeds subscriber
		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<EstimatorNode>());
	rclcpp::shutdown();

	return 0;
}
