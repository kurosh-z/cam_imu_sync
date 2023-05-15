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

  imu_raw_pub = nh.advertise<sensor_msgs::Imu>("ic_sync/imu/data_raw", 10);
  magn_pub = nh.advertise<sensor_msgs::MagneticField>("ic_sync/mag", 10);
  camtrigger_pub =
      nh.advertise<imu_cam_sync::ICtrigger>("ic_sync/camTrigger", 10);

  SerialReceivedCb serialCB =
      [&](std::uint64_t seq, ros::Time timestamp, std::array<double, 3> accel,
          std::array<double, 3> gyro, std::array<double, 3> magn,
          CamTrigger camtrig) -> void {
    this->publish_imu_cam(seq, timestamp, accel, gyro, magn, camtrig);
  };

  serial_conn = std::make_shared<IsyncSerial>(serialCB, dev_name, baudrate);

  serial_conn->connect();

  serial_conn->cmd_signal.connect(
      [this](std::string val_type, uint32_t val) -> void {
        ROS_INFO_STREAM("from signal cb: " << val_type.c_str() << "= " << val);
        this->cmd_response.val = val;
        this->cmd_response.val_type = val_type;
        this->cmd_response.set = true;
      });

  cmd_srv = nh.advertiseService("imu_cam_sync/cmd",
                                &ICamNodelet::cmd_service_handler, this);
}

void ICamNodelet::publish_imu_cam(std::uint64_t seq, ros::Time timestamp,
                                  std::array<double, 3> accel,
                                  std::array<double, 3> gyro,
                                  std::array<double, 3> magn,
                                  CamTrigger camtrig) {

  sensor_msgs::Imu imu_raw_msg;
  imu_raw_msg.header.frame_id = frame_id;
  imu_raw_msg.header.seq = seq;
  imu_raw_msg.header.stamp = timestamp;
  // Swapping the x and y values of accelerometer and gyroscope readings, so
  // that the accelerometer and gyroscope axis is aligned with magnetometer
  // axis.
  imu_raw_msg.linear_acceleration.x = -accel[1];
  imu_raw_msg.linear_acceleration.y = -accel[0];
  imu_raw_msg.linear_acceleration.z = accel[2];

  imu_raw_msg.angular_velocity.x = gyro[1];
  imu_raw_msg.angular_velocity.y = gyro[0];
  imu_raw_msg.angular_velocity.z = gyro[2];

  sensor_msgs::MagneticField magn_msg;
  magn_msg.header = imu_raw_msg.header;
  magn_msg.magnetic_field.x = magn[0];
  magn_msg.magnetic_field.y = magn[1];
  magn_msg.magnetic_field.z = magn[2];

  imu_raw_pub.publish(imu_raw_msg);
  magn_pub.publish(magn_msg);
  if (camtrig.triggered) {
    imu_cam_sync::ICtrigger cam_trig_msg;
    cam_trig_msg.cor_imu_seq = seq;
    cam_trig_msg.Header.frame_id = "";
    cam_trig_msg.Header.seq = camtrig.seq;
    cam_trig_msg.Header.stamp = timestamp;
    // TODO: fix this in message definition
    cam_trig_msg.timedelay = camtrig.timestamp;
    camtrigger_pub.publish(cam_trig_msg);
  }

  return;
}

bool ICamNodelet::cmd_service_handler(imu_cam_sync::cmd_imu::Request &req,
                                      imu_cam_sync::cmd_imu::Response &res) {

  cmd_response.set = false;

  uint8_t cmd[40] = {0};
  std::memcpy(cmd, serial_conn->CMD_GET_EXPOSURE,
              std::strlen(serial_conn->CMD_GET_EXPOSURE));
  std::memcpy(cmd + std::strlen(serial_conn->CMD_GET_EXPOSURE),
              serial_conn->END_OF_MSG, 8);
  ROS_INFO_STREAM("sending cmd: " << (char *)cmd);
  size_t len = std::strlen(serial_conn->CMD_GET_EXPOSURE) + 8;
  serial_conn->send_bytes(cmd, len);

  ros::Rate loop_rate(5);
  auto start = ros::Time::now().toSec();

  while (ros::Time::now().toSec() - start < 10.01) {
    if (cmd_response.set) {
      res.error = 0;
      res.value = cmd_response.val;
      cmd_response.set = false;
      return true;
    }
    loop_rate.sleep();
  }

  return false;
}

} // namespace ic_sync

PLUGINLIB_EXPORT_CLASS(ic_sync::ICamNodelet, nodelet::Nodelet)