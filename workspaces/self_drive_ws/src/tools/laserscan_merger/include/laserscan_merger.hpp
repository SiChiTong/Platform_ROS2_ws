#include <cstdio>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <pcl_ros/transforms.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <Eigen/Dense>
#include <memory>

// Node Name : laserscan_merger

using namespace std;
using namespace std::chrono_literals;

class LaserscanMerger : public rclcpp::Node {
public:
	LaserscanMerger();

	void pointcloud_to_laserscan(Eigen::MatrixXf points, pcl::PCLPointCloud2 *merged_cloud);

	void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg, string topic);

private:
	void laserscan_topic_parser();
	bool check_platform_init(std::string mode_name);

	laser_geometry::LaserProjection projector_;
	shared_ptr<tf2_ros::TransformListener> tf2Listener_ptr;
	shared_ptr<tf2_ros::Buffer> tf2buffer_ptr;

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;
	rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_publisher_;
	vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> scan_subscribers;

	vector<pcl::PCLPointCloud2> clouds;
	vector<string> input_topics;
	vector<bool> clouds_modified;

	double angle_min;
	double angle_max;
	double angle_increment;
	double time_increment;
	double scan_time;
	double range_min;
	double range_max;

	bool verbose;

	int timeout_count;

	string destination_frame;
	string cloud_destination_topic;
	string scan_destination_topic;
	string laserscan_topics;
	string platform_mode;

	sensor_msgs::msg::PointCloud2 pclToRos2msgs(pcl::PCLPointCloud2 pc2);
};


LaserscanMerger::LaserscanMerger() : Node("scan_merger")
{
	vector<rclcpp::Parameter> params;

	declare_parameter<string>("destination_frame", "base_link");
	declare_parameter<string>("cloud_destination_topic", "/merged_cloud");
	declare_parameter<string>("scan_destination_topic", "/merged_scan");
	declare_parameter<string>("laserscan_topics", "/scan_1 /scan_2");
	declare_parameter<string>("mode_name", "");

	declare_parameter<double>("angle_min", -3.14);
	declare_parameter<double>("angle_max", 3.14);
	declare_parameter<double>("angle_increment", 0.0058);
	declare_parameter<double>("scan_time", 0.033);
	declare_parameter<double>("range_min", 0.01);
	declare_parameter<double>("range_max", 25.0);
	declare_parameter<double>("timeout", 5.0);

	declare_parameter<bool>("verbose", false);

	get_parameter<string>("destination_frame", destination_frame);
	get_parameter<string>("cloud_destination_topic", cloud_destination_topic);
	get_parameter<string>("scan_destination_topic", scan_destination_topic);
	get_parameter<string>("laserscan_topics", laserscan_topics);
	get_parameter<string>("mode_name", platform_mode);

	get_parameter<double>("angle_min", angle_min);
	get_parameter<double>("angle_max", angle_max);
	get_parameter<double>("angle_increment", angle_increment);
	get_parameter<double>("scan_time", scan_time);
	get_parameter<double>("range_min", range_min);
	get_parameter<double>("range_max", range_max);

	double timeout_d;
	get_parameter<double>("timeout", timeout_d);
	this->timeout_count = int(timeout_d * 10);

	get_parameter<bool>("verbose", verbose);

	tf2buffer_ptr = std::make_shared<tf2_ros::Buffer>(this->get_clock());
	tf2Listener_ptr = std::make_shared<tf2_ros::TransformListener>(*tf2buffer_ptr);

	laserscan_topic_parser();

	if (input_topics.size() == 0) {
		RCLCPP_FATAL(get_logger(), "Can't Parse Any Topics!");
		rclcpp::shutdown();
	}

	laser_scan_publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(scan_destination_topic, 10);
	point_cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(cloud_destination_topic, 10);

	RCLCPP_INFO(get_logger(), "publish scan : %s  /  pointcloud2 : %s", scan_destination_topic.c_str(),
	            cloud_destination_topic.c_str());
}

void LaserscanMerger::laserscan_topic_parser()
{
	if(!check_platform_init(this->platform_mode)) return;

	bool isReceived = false;
	vector<string> tmp_input_topics;

	int count_try_get_topic = 0;

	while ((count_try_get_topic <= this->timeout_count) && (!isReceived))
	{
		auto topics = get_topic_names_and_types();

		istringstream iss(laserscan_topics);
		vector<string> target_names;
		copy(istream_iterator<string>(iss), istream_iterator<string>(), back_inserter<vector<string>>(target_names));
		auto count_target = target_names.size();

		if (this->verbose)
		{
			RCLCPP_INFO(get_logger(), "try to get topics.... %d", count_try_get_topic);
		}

		for (auto &topic : topics)
		{
			string name = topic.first;
			vector<string> types = topic.second;

			for (auto &target_name : target_names)
			{
				for (auto &type : types)
				{
					if ((target_name == name)  && (type.compare("sensor_msgs/msg/LaserScan") == 0) &&
						(find(tmp_input_topics.begin(),tmp_input_topics.end(),name) == tmp_input_topics.end())
					   )
					{
						tmp_input_topics.push_back(name);
						RCLCPP_INFO(get_logger(), "Add %s", name.c_str());
						// isReceived = true;
						if (count_target == tmp_input_topics.size()) {
							isReceived = true;
						}
					}
				}
			}
		}

		rclcpp::spin_some(this->get_node_base_interface());
		rclcpp::sleep_for(100ms);
		count_try_get_topic++;
	}

	if ((tmp_input_topics.size() != input_topics.size()) ||
	    !equal(tmp_input_topics.begin(), tmp_input_topics.end(), input_topics.begin()))
	{
		for (auto &subscriber : scan_subscribers)
		{
			subscriber.reset();
		}

		input_topics = tmp_input_topics;

		if (input_topics.size() > 0)
		{
			scan_subscribers.resize(input_topics.size());
			clouds.resize(input_topics.size());
			clouds_modified.resize(input_topics.size());

			for (int i = 0; i < input_topics.size(); ++i)
			{
				string input_topic = input_topics[i];
				RCLCPP_INFO(get_logger(), "Topic Update : %s", input_topic.c_str());

				auto func_ = [this, input_topic](const sensor_msgs::msg::LaserScan::SharedPtr msg)
				{
					if (this->verbose)
					{
						RCLCPP_INFO(get_logger(), "callback: %s", input_topic.c_str());
					}
					this->scanCallback(msg, input_topic);
				};

				scan_subscribers[i] = create_subscription<sensor_msgs::msg::LaserScan>(input_topic, rclcpp::SensorDataQoS(), func_);
				clouds_modified[i] = false;

				RCLCPP_INFO(get_logger(), "success to initialize scan_subscribers[%d]", i);
			}
		}
	}
}

bool LaserscanMerger::check_platform_init(std::string mode_name)
{
	if(mode_name != "")
	{
		int count_try_get_mode = 0;
		auto isFind = std::make_shared<bool>(false);

		auto func_ = [this, isFind](const std_msgs::msg::String::SharedPtr msg)
		{
			RCLCPP_INFO(get_logger(), "Find Platform Mode topic! %s", msg->data.c_str());

			*isFind = true;
		};

		auto sub_mode = this->create_subscription<std_msgs::msg::String>(this->platform_mode, rclcpp::SensorDataQoS(), func_);

		while (!(*isFind) && (count_try_get_mode <= this->timeout_count))
		{
			if (this->verbose)
			{
				RCLCPP_INFO(get_logger(), "try to get platform mode.... %d", count_try_get_mode);
			}
			rclcpp::spin_some(this->get_node_base_interface());
			rclcpp::sleep_for(100ms);
			count_try_get_mode++;
		}

		if(!(*isFind))
		{
			RCLCPP_INFO(get_logger(), "Can't Find platform mode topic!");
			return false;
		}

		sub_mode.reset();
		return true;
	}
}

void LaserscanMerger::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan, string topic) {
	if (clouds.size() != 0) {

		sensor_msgs::msg::PointCloud2 tmpCloud1, tmpCloud2;

		if (this->verbose) RCLCPP_INFO(get_logger(), "%s : try to Transform...", topic.c_str());

		try {
			this->tf2buffer_ptr->lookupTransform(scan->header.frame_id, destination_frame, scan->header.stamp,
			                                     tf2::Duration(1));
			projector_.transformLaserScanToPointCloud(scan->header.frame_id, *scan, tmpCloud1, *tf2buffer_ptr, -1.0,
			                                          laser_geometry::channel_option::Distance);
			pcl_ros::transformPointCloud(destination_frame, tmpCloud1, tmpCloud2, *tf2buffer_ptr);
		}
		catch (tf2::TransformException &e) {
			RCLCPP_INFO(get_logger(), "%s", e.what());
		}

		if (this->verbose) RCLCPP_INFO(get_logger(), "%s : try to conversion...(clouds)", topic.c_str());

		for (int i = 0; i < input_topics.size(); ++i) {
			if (topic.compare(input_topics[i]) == 0) {
				try {
					pcl_conversions::toPCL(tmpCloud2, clouds[i]);
					clouds_modified[i] = true;
				}
				catch (const std::exception &e) {
					RCLCPP_INFO(get_logger(), "%s", e.what());
				}
			}
		}

		int totalClouds = 0;
		for (int i = 0; i < clouds_modified.size(); ++i)
			if (clouds_modified[i])
				++totalClouds;

		if (totalClouds == clouds_modified.size()) {
			if (this->verbose) RCLCPP_INFO(get_logger(), "%s : try to merging...(merged_cloud)", topic.c_str());

			pcl::PCLPointCloud2 merged_cloud = clouds[0];
			clouds_modified[0] = false;

			for (int i = 1; i < clouds_modified.size(); ++i) {
				pcl::concatenate(merged_cloud, clouds[i], merged_cloud);
				// pcl::concatenatePointCloud(merged_cloud, clouds[i], merged_cloud);
				clouds_modified[i] = false;
			}

			sensor_msgs::msg::PointCloud2 merged_cloud_ros;

			pcl_conversions::fromPCL(merged_cloud, merged_cloud_ros);

			point_cloud_publisher_->publish(merged_cloud_ros);

			Eigen::MatrixXf points;
			getPointCloudAsEigen(merged_cloud, points);

			pointcloud_to_laserscan(points, &merged_cloud);
		}
	}
}

void LaserscanMerger::pointcloud_to_laserscan(const Eigen::MatrixXf points, pcl::PCLPointCloud2 *merged_cloud)
{
	sensor_msgs::msg::LaserScan::SharedPtr output(new sensor_msgs::msg::LaserScan());

	output->header = pcl_conversions::fromPCL(merged_cloud->header);
	output->angle_min = this->angle_min;
	output->angle_max = this->angle_max;
	output->angle_increment = this->angle_increment;
	output->time_increment = this->time_increment;
	output->scan_time = this->scan_time;
	output->range_min = this->range_min;
	output->range_max = this->range_max;

	uint32_t ranges_size = std::ceil((output->angle_max - output->angle_min) / output->angle_increment);

	output->ranges.assign(ranges_size, output->range_max + 1.0);

	for (int i = 0; i < points.cols(); i++) {
		const float &x = points(0, i);
		const float &y = points(1, i);
		const float &z = points(2, i);

		if (std::isnan(x) || std::isnan(y) || std::isnan(z)) {
			if (this->verbose) {
				RCLCPP_INFO(get_logger(), "rejected for nan in point(%f, %f, %f)\n", x, y, z);
			}

			continue;
		}

		double range_sq = y * y + x * x;
		double range_min_sq_ = output->range_min * output->range_min;
		if (range_sq < range_min_sq_) {
			if (this->verbose) {
				RCLCPP_INFO(get_logger(), "rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range_sq,
				            range_min_sq_, x, y, z);
			}
			continue;
		}

		double angle = atan2(y, x);
		if (angle < output->angle_min || angle > output->angle_max) {
			if (this->verbose) {
				RCLCPP_INFO(get_logger(), "rejected for angle %f not in range (%f, %f)\n", angle, output->angle_min,
				            output->angle_max);
			}
			continue;
		}
		int index = (angle - output->angle_min) / output->angle_increment;

		// Take Minimum Only Value...
		if (output->ranges[index] * output->ranges[index] > range_sq)
			output->ranges[index] = sqrt(range_sq);
	}

	laser_scan_publisher_->publish(*output);
}