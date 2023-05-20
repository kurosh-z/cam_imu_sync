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
  NODELET_INFO("ICamNodelet - %s", __FUNCTION__);

  ros::NodeHandle &nh = getNodeHandle();
  ros::NodeHandle &local_nh = getPrivateNodeHandle();

  local_nh.param<std::string>("dev_name", dev_name, DEFAULT_DEVICE_NAME);
  local_nh.param<int>("baudrate", baudrate, DEFAULT_BAUDRATE);
  ROS_INFO_STREAM("dev_name: " << dev_name << ", baudrate: " << baudrate);

  imu_raw_pub = nh.advertise<sensor_msgs::Imu>("imu_cam_sync/imu/data_raw", 10);
  magn_pub = nh.advertise<sensor_msgs::MagneticField>("imu_cam_sync/mag", 10);
  camtrigger_pub = nh.advertise<imu_cam_msgs::ICtrigger>(
      "imu_cam_sync/cam_trigger_stamp", 10);

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
    imu_cam_msgs::ICtrigger cam_trig_msg;
    cam_trig_msg.corr_imu_seq = seq;
    cam_trig_msg.Header.frame_id = "";
    cam_trig_msg.Header.seq = camtrig.seq;
    cam_trig_msg.Header.stamp = timestamp;
    camtrigger_pub.publish(cam_trig_msg);
  }

  return;
}

bool ICamNodelet::cmd_service_handler(imu_cam_msgs::cmd_imu::Request &req,
                                      imu_cam_msgs::cmd_imu::Response &res) {

  std::lock_guard<std::recursive_mutex> lock(cmd_srv_mutex);
  cmd_response.set = false;
  uint8_t cmd[50] = {0};

  if (req.cmd_type == serial_conn->CMD_GET_EXPOSURE) {
    size_t len = std::strlen(serial_conn->CMD_GET_EXPOSURE);
    std::memcpy(cmd, serial_conn->CMD_GET_EXPOSURE, len);
    ROS_INFO_STREAM("sending cmd: " << (char *)cmd);

    serial_conn->send_bytes(cmd, len);

  } else if (req.cmd_type == serial_conn->CMD_SET_EXPOSURE) {
    size_t len = std::strlen(serial_conn->CMD_SET_EXPOSURE);
    std::memcpy(cmd, serial_conn->CMD_SET_EXPOSURE, len); // 16 bytes
    utils::sys_put_be32(req.cmd_val, &cmd[16]);           // + 4 bytes
    ROS_INFO_STREAM("sending cmd: " << (char *)cmd);
    serial_conn->send_bytes(cmd, len + 4);

  } else if (req.cmd_type == serial_conn->CMD_GET_TRIGGER) {
    size_t len = std::strlen(serial_conn->CMD_GET_TRIGGER);
    std::memcpy(cmd, serial_conn->CMD_GET_TRIGGER, len);
    ROS_INFO_STREAM("sending cmd: " << (char *)cmd);
    serial_conn->send_bytes(cmd, len);

  } else if (req.cmd_type == serial_conn->CMD_SET_TRIGGER) {
    size_t len = std::strlen(serial_conn->CMD_SET_TRIGGER);

    std::memcpy(cmd, serial_conn->CMD_SET_TRIGGER, len); // 15 bytes
    cmd[15] = req.cmd_val;                               // + 1 bytes
    ROS_INFO_STREAM("sending cmd: " << (char *)cmd);
    serial_conn->send_bytes(cmd, len + 1);
  }

  else {
    res.error = CMD_REPS_UNKOWN;
    res.response_type = "ERROR";
    return true;
  }

  ros::Rate loop_rate(5);
  auto start = ros::Time::now().toSec();

  while (ros::Time::now().toSec() - start < 4) {
    if (cmd_response.set) {
      res.error = 0;
      res.value = cmd_response.val;
      res.response_type = cmd_response.val_type;
      return true;
    }
    loop_rate.sleep();
  }

  res.error = CMD_RESP_TIMED_OUT;
  res.response_type = "ERROR";
  return true;
}

} // namespace ic_sync

PLUGINLIB_EXPORT_CLASS(ic_sync::ICamNodelet, nodelet::Nodelet)