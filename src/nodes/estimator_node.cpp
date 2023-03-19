#include "rclcpp/rclcpp.hpp"
#include "ackermann_state_estimator/estimator_node.hpp"

EstimatorNode::EstimatorNode() : Node("estimator_node") {
	publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("twist", 10);
	ackermann_subscriber_ = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(
			"ackermann", 10, std::bind(&EstimatorNode::ackermann_callback, this, std::placeholders::_1));

	// subscribe to wheel speeds
	left_front_wheel_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
			"left_front_wheel", 10, std::bind(&EstimatorNode::left_front_wheel_callback, this, std::placeholders::_1));
	right_front_wheel_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
			"right_front_wheel", 10, std::bind(&EstimatorNode::right_front_wheel_callback, this, std::placeholders::_1));
	left_back_wheel_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
			"left_back_wheel", 10, std::bind(&EstimatorNode::left_back_wheel_callback, this, std::placeholders::_1));
	right_back_wheel_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
			"right_back_wheel", 10, std::bind(&EstimatorNode::right_back_wheel_callback, this, std::placeholders::_1));	
}

void EstimatorNode::ackermann_callback(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg) {
	this->cur_ackermann = *msg;
	this->ackermann_received = true;

	// TODO: check slip

	// TODO: calculate twist

	// TODO: publish twist
}

void EstimatorNode::left_front_wheel_callback(const std_msgs::msg::Float64::SharedPtr msg) {
	this->wheel_speeds[0][0].measured = msg->data;
}

void EstimatorNode::right_front_wheel_callback(const std_msgs::msg::Float64::SharedPtr msg) {
	this->wheel_speeds[1][0].measured = msg->data;
}

void EstimatorNode::left_back_wheel_callback(const std_msgs::msg::Float64::SharedPtr msg) {
	this->wheel_speeds[0][1].measured = msg->data;
}

void EstimatorNode::right_back_wheel_callback(const std_msgs::msg::Float64::SharedPtr msg) {
	this->wheel_speeds[1][1].measured = msg->data;
}

// get the wheel speed of the car while driving on a straight line
double get_wheel_speed_from_drive(double drive_speed, double wheel_radius) {
	return drive_speed / wheel_radius;
}

// get a wheel speed from the drive speed, the wheel turning radius and the turning radius
double get_wheel_speed_from_radiuses(double wheel_traj_radius, double turning_radius, double drive_speed) {
	return drive_speed * wheel_traj_radius / turning_radius;
}

void EstimatorNode::get_cur_expected_wheel_speeds() {

	if(this->cur_ackermann.steering_angle == 0) {
		// if the steering angle is 0, the turning radius is infinite
		#pragma omp parallel for collapse(2)
		for(int i = 0; i < 2; i++) {
			for(int j = 0; j < 2; j++) {
				this->wheel_speeds[i][j].expected = get_wheel_speed_from_drive(this->cur_ackermann.speed, WHEEL_RADIUS);
			}
		}
	} else {
		double turning_radius = WHEEL_BASE / tan(this->cur_ackermann.steering_angle);
		double radius_diff = TRACK / 2 * sin(this->cur_ackermann.steering_angle);

		int multiplier = this->cur_ackermann.steering_angle > 0 ? 1 : -1;

		// calculate the left and right turning radiuses, based on the steering angle
		#pragma omp parallel for collapse(2)
		for(int i = 0; i < 2; i++) {
			double wheel_traj_radius = turning_radius + (multiplier * radius_diff);
			for(int j = 0; j < 2; j++) {
				this->wheel_speeds[i][j].expected = get_wheel_speed_from_radiuses(wheel_traj_radius, turning_radius, this->cur_ackermann.speed);
			}
			multiplier *= -1;
		}
	}
}

void EstimatorNode::get_cur_wheel_speeds_ratio() {

	#pragma omp parallel for collapse(2)
	for(int i = 0; i < 2; i++) {
		for(int j = 0; j < 2; j++) {
			this->wheel_speeds[i][j].ratio = this->wheel_speeds[i][j].measured / this->wheel_speeds[i][j].expected;
		}
	}
}

bool EstimatorNode::are_all_wheel_speeds_set() {
	for(int i = 0; i < 2; i++) {
		for(int j = 0; j < 2; j++) {
			if(!this->wheel_speeds[i][j].set) {
				return false;
			}
		}
	}
	return true;
}

int main(int argc, char *argv[]) {

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<EstimatorNode>());
	rclcpp::shutdown();

	return 0;
}
