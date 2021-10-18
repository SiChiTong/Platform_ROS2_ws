#include <laserscan_merger.hpp>

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);

	rclcpp::spin(make_shared<LaserscanMerger>());

	rclcpp::shutdown();
	return 0;
}
