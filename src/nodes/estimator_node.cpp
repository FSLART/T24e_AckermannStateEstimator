#include "rclcpp/rclcpp.hpp"
#include "ackermann_state_estimator/estimator_node.hpp"

EstimatorNode::EstimatorNode() : Node("estimator_node") {
	publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("twist", 10);
	ackermann_subscriber_ = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(
			"ackermann", 10, std::bind(&EstimatorNode::ackermann_callback, this, std::placeholders::_1));
}

void EstimatorNode::odom_callback() {
	// TODO called on each jtimer tick to publish twist
}

void EstimatorNode::ackermann_callback(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg) const {
	RCLCPP_INFO(this->get_logger(), "New ackermann message");
}

int main(int argc, char *argv[]) {

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<EstimatorNode>());
	rclcpp::shutdown();

	return 0;
}
