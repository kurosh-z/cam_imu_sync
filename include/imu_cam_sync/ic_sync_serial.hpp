#pragma once
#ifndef ICAM_SYNC_SERIAL_HPP_
#define ICAM_SYNC_SERIAL_HPP_

#include <atomic>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <deque>
#include <iomanip>
#include <iostream>
#include <linux/serial.h>
#include <thread>

#include "imu_cam_sync/thread_utils.hpp"

namespace ic_sync {
// : public std::enable_shared_from_this<IsyncSerial>
class IsyncSerial {

public:
  static constexpr unsigned int RX_MAX_SIZE = 256;
  static constexpr auto DEFAULT_DEVICE_NAME = "/dev/ttyACM0";
  static constexpr auto DEFAULT_BAUDRATE = 115200;

  IsyncSerial(std::string dev_name, unsigned baudrate);
  ~IsyncSerial();
  void connect();
  void close();

protected:
  std::string dev_name;
  uint32_t baudrate;

  inline bool is_open() {
    { return serial_dev.is_open(); }
  }

  boost::asio::io_service io_service;
  std::thread io_thread;
  boost::asio::serial_port serial_dev;
  std::array<uint8_t, RX_MAX_SIZE> rx_buf;

  std::atomic<bool> tx_in_progress;
  std::recursive_mutex mutex;
  boost::asio::streambuf received_data_buf;

  void do_read();
  void on_read(std::error_code error, size_t bytes_transferred);

  void parse_buff(uint8_t *buf, const size_t buffsize, size_t bytes_recieved);
}; // end of class IsyncSerial

} // namespace ic_sync

#endif