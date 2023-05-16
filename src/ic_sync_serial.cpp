
#include "imu_cam_sync/ic_sync_serial.hpp"

namespace ic_sync {
IsyncSerial::IsyncSerial(SerialReceivedCb recieved_cb,
                         std::string dev_name = DEFAULT_DEVICE_NAME,
                         unsigned baudrate = DEFAULT_BAUDRATE)
    : io_service(), serial_dev(io_service), tx_in_progress(false), rx_buf{} {
  using SPB = boost::asio::serial_port_base;
  this->serial_recieved_cb = recieved_cb;
  try {
    serial_dev.open(dev_name);

    // Set baudrate and 8N1 mode
    serial_dev.set_option(SPB::baud_rate(baudrate));
    serial_dev.set_option(SPB::character_size(8));
    serial_dev.set_option(SPB::parity(SPB::parity::none));
    serial_dev.set_option(SPB::stop_bits(SPB::stop_bits::one));
    serial_dev.set_option(SPB::flow_control(SPB::flow_control::none));

    int fd = serial_dev.native_handle();

    struct serial_struct ser_info;
    ioctl(fd, TIOCGSERIAL, &ser_info);

    ser_info.flags |= ASYNC_LOW_LATENCY;

    ioctl(fd, TIOCSSERIAL, &ser_info);

  } catch (boost::system::system_error const &err) {
    std::cout << "ERROR: [serial connection] " << err.what() << std::endl;
    throw;
  }
}
IsyncSerial::~IsyncSerial() {
  close();
  std::cout << "serial connection closed" << std::endl;
  return;
}

void IsyncSerial::connect() {
  io_service.post(std::bind(&IsyncSerial::do_read, this));
  // run io_serice for async io
  // io_thread =
  //     std::thread([this]() { utils::set_this_thread_name("ic_serial%zu", 1);
  //     });

  io_thread =
      std::thread(boost::bind(&boost::asio::io_service::run, &io_service));
}
void IsyncSerial::on_read(std::error_code error, size_t bytes_transferred) {
  if (error) {
    std::cout << "error reading data: " << error.message().c_str() << std::endl;
    close();

    return;
  }

  // parse_buff(received_data_buf.data(), rx_buf.size(), bytes_transferred);
  parse_buf(bytes_transferred);

  do_read();
}

void IsyncSerial::do_read() {

  boost::asio::async_read_until(
      serial_dev, received_data_buf, END_OF_MSG,
      boost::bind(&IsyncSerial::on_read, this, boost::asio::placeholders::error,
                  boost::asio::placeholders::bytes_transferred));
}

void IsyncSerial::close() {
  std::lock_guard<std::recursive_mutex> lock(mutex);
  if (!is_open()) {
    return;
  }

  serial_dev.cancel();
  serial_dev.close();
  io_service.stop();

  if (io_thread.joinable()) {
    io_thread.join();
  }
  io_service.reset();
}
void IsyncSerial::parse_buf(size_t recieved_bytes) {
  auto timestamp = ros::Time::now();
  auto buffer = received_data_buf.data();
  auto first = boost::asio::buffer_cast<const uint8_t *>(buffer);
  auto last = first + recieved_bytes;

  if (std::strncmp((const char *)first, "IMU", 3) == 0) {

    // else it should be imu message
    uint64_t seq = utils::sys_get_be64(first + 3);        // 8 bytes
    uint64_t imu_stamp = utils::sys_get_be64(first + 11); // 8 bytes
    std::array<double, 3> accel = {0, 0, 0};
    std::array<double, 3> gyro = {0, 0, 0};
    std::array<double, 3> magn = {0, 0, 0};
    for (int i = 0; i < 3; i++) {

      switch (i) {
      case 0:
        accel[0] = (double)(utils::decode_int32(first + 3 + 16 + i * 24)) +
                   (double)(utils::decode_int32(first + 3 + 16 + i * 24 + 4)) /
                       1000000;
        accel[1] = (double)(utils::decode_int32(first + 3 + 24 + i * 24)) +
                   (double)(utils::decode_int32(first + 3 + 24 + i * 24 + 4)) /
                       1000000;
        accel[2] = (double)(utils::decode_int32(first + 3 + 32 + i * 24)) +
                   (double)(utils::decode_int32(first + 3 + 32 + i * 24 + 4)) /
                       1000000;
        break;
      case 1:
        gyro[0] = (double)(utils::decode_int32(first + 3 + 16 + i * 24)) +
                  (double)(utils::decode_int32(first + 3 + 16 + i * 24 + 4)) /
                      1000000;
        gyro[1] = (double)(utils::decode_int32(first + 3 + 24 + i * 24)) +
                  (double)(utils::decode_int32(first + 3 + 24 + i * 24 + 4)) /
                      1000000;
        gyro[2] = (double)(utils::decode_int32(first + 3 + 32 + i * 24)) +
                  (double)(utils::decode_int32(first + 3 + 32 + i * 24 + 4)) /
                      1000000;
        break;
      case 2:
        magn[0] = (double)(utils::decode_int32(first + 3 + 16 + i * 24)) +
                  (double)(utils::decode_int32(first + 3 + 16 + i * 24 + 4)) /
                      1000000;
        magn[1] = (double)(utils::decode_int32(first + 3 + 24 + i * 24)) +
                  (double)(utils::decode_int32(first + 3 + 24 + i * 24 + 4)) /
                      1000000;
        magn[2] = (double)(utils::decode_int32(first + 3 + 32 + i * 24)) +
                  (double)(utils::decode_int32(first + 3 + 32 + i * 24 + 4)) /
                      1000000;
        break;
      }
    }
    uint8_t triggered = first[91];

    CamTrigger camtrig = {
        .triggered = triggered ? true : false,
        .seq = 0,
        .timestamp = 0,
    };
    camtrig.seq = utils::sys_get_be64(first + 92);
    camtrig.timestamp = utils::sys_get_be64(first + 100);
    if (seq % 400 == 0) {
      std::cout << "recieved bytes: " << recieved_bytes << std::endl;
      std::cout << "seq:" << seq << " imu_stamp: " << imu_stamp << std::endl;

      std::cout << "accel: " << accel[0] << " , " << accel[1] << " , "
                << accel[2] << std::endl;

      std::cout << "gyro: " << gyro[0] << " , " << gyro[1] << " , " << gyro[2]
                << std::endl;

      std::cout << "magn: " << magn[0] << " , " << magn[1] << " , " << magn[2]
                << std::endl;
      if (triggered) {
        uint32_t trigger_seq = utils::sys_get_be64(first + 92);
        uint32_t trigger_timestamp = utils::sys_get_be64(first + 100);
        camtrig.seq = trigger_seq;
        camtrig.timestamp = trigger_timestamp;
        std::cout << "trig_seq: " << trigger_seq << std::endl;
        std::cout << "trig_timestamp: " << trigger_timestamp << std::endl;
      }
      std::cout << "\n---------------------------------\n\n" << std::endl;
    }

    if (serial_recieved_cb) {
      serial_recieved_cb(seq, timestamp, accel, gyro, magn, camtrig);
    }
  }

  else if (std::strncmp((const char *)first, RESP_GET_EXPOSURE, 17) == 0) {
    std::cout << "-----------------------------\n";
    uint16_t exposure_ms = utils::sys_get_be32(&first[17]);
    ROS_INFO_STREAM("got get exposure_ms response: " << exposure_ms);
    cmd_signal(RESP_GET_EXPOSURE, exposure_ms);
  } else if (std::strncmp((const char *)first, RESP_SET_EXPOSURE, 17) == 0) {
    std::cout << "-----------------------------\n";
    uint16_t exposure_ms = utils::sys_get_be32(&first[17]);
    ROS_INFO_STREAM("got set exposure_ms resp: " << exposure_ms);
    cmd_signal(RESP_SET_EXPOSURE, exposure_ms);
  } else if (std::strncmp((const char *)first, RESP_GET_TRIGGER, 16) == 0) {
    std::cout << "-----------------------------\n";
    uint8_t trigger_state = first[16];
    ROS_INFO_STREAM("got get trigger state resp: " << trigger_state);
    cmd_signal(RESP_GET_TRIGGER, trigger_state);
  } else if (std::strncmp((const char *)first, RESP_SET_TRIGGER, 16) == 0) {
    std::cout << "-----------------------------\n";
    uint8_t trigger_state = first[16];
    ROS_INFO_STREAM("got set trigger state resp: " << trigger_state);
    cmd_signal(RESP_SET_TRIGGER, trigger_state);
  } else {
    ROS_WARN("Got unknown response! ");
  }
  received_data_buf.consume(recieved_bytes);

  // std::cout << "recieved data" << std::endl;
  // for (auto elp = first; elp < last; elp++) {
  //   std::cout << std::setfill('0') << std::setw(2) << std::hex
  //             << (0xff & (*elp)) << "-";
  // }
}
void IsyncSerial::send_bytes(uint8_t *msg, size_t len) {
  if (!is_open()) {
    return;
  }

  if (msg == nullptr) {
    return;
  }

  {
    std::lock_guard<std::recursive_mutex> lock(mutex);

    if (wr_buf.size() >= MAX_WRQ_SIZE)
      throw std::length_error("IsyncSerial::send_bytes: wr_buf queue overflow");

    auto tx_buf = new std::array<uint8_t, 128>();

    std::memcpy(tx_buf->begin(), msg, len);
    // add message ending characters
    std::memcpy(tx_buf->begin() + len, END_OF_MSG, 8);
    SharedBufferPtr_t tx_ptr(tx_buf);
    wr_buf.push_back(tx_ptr);
  }

  io_service.post(boost::bind(&IsyncSerial::do_write, this, true, len + 8));

  // io_thread =
  //     std::thread([this]() { utils::set_this_thread_name("ic_serial%zu", 1);
  //     });
  // io_service.run();
}
void IsyncSerial::do_write(bool check_tx_state, size_t len) {
  if (check_tx_state && tx_in_progress)
    return;
  tx_in_progress = true;
  boost::asio::async_write(
      serial_dev, boost::asio::buffer(*wr_buf.front(), len),
      boost::asio::transfer_all(),
      boost::bind(&IsyncSerial::on_write_ends, this,
                  boost::asio::placeholders::error,
                  boost::asio::placeholders::bytes_transferred));
}

void IsyncSerial::on_write_ends(const boost::system::error_code error,
                                size_t bytes_transfered) {

  if (error) {
    std::cout << "error reading data: " << error.message().c_str() << std::endl;
    close();
    return;
  }

  wr_buf.pop_front();
  if (!wr_buf.empty())
    do_write(false, wr_buf.front()->size());
  else
    tx_in_progress = false;
}

} // namespace ic_sync