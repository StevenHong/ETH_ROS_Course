#include <cmath>
#include <algorithm>
#include <smb_highlevel_controller/SmbHighlevelController.hpp>

namespace smb_highlevel_controller {

SmbHighlevelController::SmbHighlevelController(ros::NodeHandle &nodeHandle) :
		nodeHandle_(nodeHandle), subscriberQueueSize_(10), scanTopic_("/scan") {
	if (!readParameters()) {
		ROS_ERROR("Could not read parameters.");
		ros::requestShutdown();
	}
	scanSubscriber_ = nodeHandle_.subscribe(scanTopic_, subscriberQueueSize_,
			&SmbHighlevelController::scanCallback, this);
	pclSubscriber_ = nodeHandle_.subscribe("/rslidar_points",1, &SmbHighlevelController::pointcloudCallback, this);
}

SmbHighlevelController::~SmbHighlevelController() {
}

bool SmbHighlevelController::readParameters() {
	bool success = true;
	success &= nodeHandle_.getParam(
			"/smb_highlevel_controller/scan_subscriber_topic_name", scanTopic_);
	success &= nodeHandle_.getParam(
			"/smb_highlevel_controller/scan_subscriber_queue_size",
			subscriberQueueSize_);
	return success;
}

/* bonus task solution */
void SmbHighlevelController::pointcloudCallback(const sensor_msgs::PointCloud2 &msg){
	ROS_INFO_STREAM_THROTTLE(2.0, "num poins in 3D cloud: " << msg.data.size());
}

void SmbHighlevelController::scanCallback(
		const sensor_msgs::LaserScan::ConstPtr &msg) {
	double min = msg->range_max;
	for (int i = 0; i < msg->ranges.size(); ++i) {
		if (msg->ranges[i] < min)
			min = msg->ranges[i];
	}
	ROS_INFO_STREAM_THROTTLE(2.0,"Minimum Range: " << min);
}

}  // namespace smb_highlevel_controller
