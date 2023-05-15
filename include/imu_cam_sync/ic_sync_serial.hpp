#pragma once
#ifndef ICAM_SYNC_SERIAL_HPP_
#define ICAM_SYNC_SERIAL_HPP_

#include <atomic>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/signals2.hpp>
#include <boost/thread/mutex.hpp>
#include <deque>
#include <iomanip>
#include <iostream>
#include <linux/serial.h>
#include <ros/ros.h>
#include <thread>

#include "imu_cam_sync/bytes_utils.hpp"
#include "imu_cam_sync/thread_utils.hpp"

namespace ic_sync {

typedef struct CamTrigger {
  bool triggered;
  uint64_t seq;
  uint64_t timestamp;
} CamTrigger;

typedef std::function<void(
    std::uint64_t seq, ros::Time timestamp, std::array<double, 3> accel,
    std::array<double, 3> gyro, std::array<double, 3> magn, CamTrigger camtrig)>
    SerialReceivedCb;

class IsyncSerial {

public:
  static constexpr unsigned int RX_MAX_SIZE = 256;
  static constexpr unsigned int MAX_WRQ_SIZE = 3;
  static constexpr auto DEFAULT_DEVICE_NAME = "/dev/ttyACM0";
  static constexpr auto DEFAULT_BAUDRATE = 115200;
  static constexpr auto END_OF_MSG = "ENDOFMSG";

  // CMD:
  static constexpr auto CMD_SET_EXPOSURE = "CMD_SET_EXPOSURE"; // uint16 value
  static constexpr auto CMD_GET_EXPOSURE = "CMD_GET_EXPOSURE"; // uint16 value
  static constexpr auto CMD_SET_TRIGGER = "CMD_SET_TRIGGER";   // uint8 value
  static constexpr auto CMD_GET_TRIGGER = "CMD_GET_TRIGGER";   // uint8 value
  static constexpr auto CMD_GET_ERROR = "CMD_GET_ERROR";       // uint8 value
  // Response

  static constexpr auto RESP_SET_EXPOSURE = "RESP_SET_EXPOSURE"; // uint16 value
  static constexpr auto RESP_GET_EXPOSURE = "RESP_GET_EXPOSURE"; // uint16 value
  static constexpr auto RESP_SET_TRIGGER = "RESP_SET_TRIGGER";   // uint8 value
  static constexpr auto RESP_GET_TRIGGER = "RESP_GET_TRIGGER";   // uint8 value
  static constexpr auto RESP_GET_ERROR = "RESP_GET_ERROR";       // uint8 value

  IsyncSerial(SerialReceivedCb recieved_cb, std::string dev_name,
              unsigned baudrate);
  ~IsyncSerial();
  void connect();
  void close();
  void send_bytes(uint8_t *msg, size_t len);
  boost::signals2::signal<void(std::string val_type, uint32_t val)> cmd_signal;

protected:
  std::string dev_name;
  uint32_t baudrate;
  SerialReceivedCb serial_recieved_cb;

  inline bool is_open() {
    { return serial_dev.is_open(); }
  }

  boost::asio::io_service io_service;
  std::thread io_thread;
  boost::asio::serial_port serial_dev;
  std::array<uint8_t, RX_MAX_SIZE> rx_buf;
  typedef std::shared_ptr<std::array<uint8_t, 128>> SharedBufferPtr_t;
  std::deque<SharedBufferPtr_t> wr_buf;

  std::atomic<bool> tx_in_progress;
  std::recursive_mutex mutex;
  boost::asio::streambuf received_data_buf;

  void do_read();
  void on_read(std::error_code error, size_t bytes_transferred);
  void parse_buf(size_t bytes_recieved);

  void do_write(bool check_tx_state, size_t len);
  void on_write_ends(const boost::system::error_code, size_t bytes_transfered);

}; // end of class IsyncSerial

} // namespace ic_sync

#endif