#pragma once
#ifndef ICAM_SYNC_NODELET_HPP_
#define ICAM_SYNC_NODELET_HPP_
#include <atomic>
#include <boost/asio.hpp>
#include <boost/thread/mutex.hpp>
#include <deque>
#include <imu_cam_sync/ICtrigger.h>
#include <imu_cam_sync/cmd_imu.h>
#include <imu_cam_sync/ic_sync_serial.hpp>
#include <iostream>
#include <linux/serial.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <thread>

namespace ic_sync {
#define DEFAULT_BAUDRATE 115200

class ICamNodelet : public nodelet::Nodelet {
public:
  static constexpr auto DEFAULT_DEVICE_NAME = "/dev/ttyACM0";
  // static constexpr int DEFAULT_BAUDRATE = 115200;

  ICamNodelet();
  ~ICamNodelet();
  virtual void onInit();
  void publish_imu_cam(std::uint64_t seq, ros::Time timestamp,
                       std::array<double, 3> accel, std::array<double, 3> gyro,
                       std::array<double, 3> magn, CamTrigger camtrig);

protected:
  std::string frame_id;
  ros::Publisher imu_raw_pub;
  ros::Publisher magn_pub;
  ros::Publisher camtrigger_pub;
  std::string dev_name;
  SerialReceivedCb serial_received_cb;
  int baudrate;
  std::shared_ptr<IsyncSerial> serial_conn;

  struct cmd_response_t {
    uint32_t val;
    std::string val_type;
    bool set;
  };
  struct cmd_response_t cmd_response;
  ros::ServiceServer cmd_srv;
  bool cmd_service_handler(imu_cam_sync::cmd_imu::Request &req,
                           imu_cam_sync::cmd_imu::Response &res);
};
} // namespace ic_sync

#endif