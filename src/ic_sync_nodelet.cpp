
#include <imu_cam_sync/ic_sync_nodelet.hpp>

#include <iomanip>
#include <pluginlib/class_list_macros.h>

namespace ic_sync {

ICamNodelet::ICamNodelet() { ROS_INFO("ICamNodelet Constructor"); }
ICamNodelet::~ICamNodelet() {
  ROS_INFO("ICamNodelet Destructor");
  serial_conn->close();
}

void ICamNodelet::onInit() {
  ROS_INFO("=================================================================");
  NODELET_INFO("VCamNodelet - %s", __FUNCTION__);

  ros::NodeHandle &nh = getNodeHandle();
  ros::NodeHandle &local_nh = getPrivateNodeHandle();

  local_nh.param<std::string>("dev_name", dev_name, DEFAULT_DEVICE_NAME);
  local_nh.param<int>("baudrate", baudrate, DEFAULT_BAUDRATE);

  ROS_INFO_STREAM("dev_name: " << dev_name << ", baudrate: " << baudrate);

  serial_conn = std::make_shared<IsyncSerial>(dev_name, baudrate);
  serial_conn->connect();
}

} // namespace ic_sync

PLUGINLIB_EXPORT_CLASS(ic_sync::ICamNodelet, nodelet::Nodelet)