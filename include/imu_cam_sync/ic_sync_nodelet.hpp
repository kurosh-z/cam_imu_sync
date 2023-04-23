#pragma once
#ifndef ICAM_SYNC_NODELET_HPP_
#define ICAM_SYNC_NODELET_HPP_

#include "imu_cam_sync/ic_sync_serial.hpp"
#include <atomic>
#include <boost/asio.hpp>
#include <boost/thread/mutex.hpp>
#include <deque>
#include <iostream>
#include <linux/serial.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
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

protected:
  std::string dev_name;
  int baudrate;
  std::shared_ptr<IsyncSerial> serial_conn;
};
} // namespace ic_sync

#endif