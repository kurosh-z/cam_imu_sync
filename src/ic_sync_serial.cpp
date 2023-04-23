
#include "imu_cam_sync/ic_sync_serial.hpp"

namespace ic_sync {
IsyncSerial::IsyncSerial(std::string dev_name = DEFAULT_DEVICE_NAME,
                         unsigned baudrate = DEFAULT_BAUDRATE)
    : io_service(), serial_dev(io_service), tx_in_progress(false), rx_buf{} {
  using SPB = boost::asio::serial_port_base;

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
  io_thread =
      std::thread([this]() { utils::set_this_thread_name("ic_serial%zu", 1); });
  io_service.run();
}
void IsyncSerial::on_read(std::error_code error, size_t bytes_transferred) {
  if (error) {
    std::cout << "error reading data: " << error.message().c_str() << std::endl;
    close();
    return;
  }
  std::cout << "data received: " << bytes_transferred << std::endl;
  std::istream is(&received_data_buf);
  std::string line;
  std::getline(is, line);
  std::cout << line << std::endl;

  // parse_buff(line, rx_buf.size(), bytes_transferred);

  std::cout << std::endl;
  do_read();
}
void IsyncSerial::do_read() {
  // std::cout << "do_read: " << std::endl;
  // auto sthis = shared_from_this();

  boost::asio::async_read_until(
      serial_dev, received_data_buf, "\n",
      boost::bind(&IsyncSerial::on_read, this, boost::asio::placeholders::error,
                  boost::asio::placeholders::bytes_transferred));
  // boost::asio::async_read_until(
  //     serial_dev, received_data,
  //     [sthis](std::error_code error, size_t bytes_transferred) {
  //       if (error) {
  //         std::cout << "error reading data: " << error.message().c_str()
  //                   << std::endl;
  //         sthis->close();
  //         return;
  //       }
  //       std::cout << "data received: " << bytes_transferred << std::endl;
  //       sthis->parse_buff(sthis->rx_buf.data(), sthis->rx_buf.size(),
  //                         bytes_transferred);

  //       std::cout << std::endl;

  //       sthis->do_read();
  //     });

  // serial_dev.async_read_some(
  //     boost::asio::buffer(rx_buf),
  //     [sthis](std::error_code error, size_t bytes_transferred) {
  //       if (error) {
  //         std::cout << "error reading data: " << error.message().c_str()
  //                   << std::endl;
  //         sthis->close();
  //         return;
  //       }
  //       std::cout << "data received: " << bytes_transferred << std::endl;
  //       sthis->parse_buff(sthis->rx_buf.data(), sthis->rx_buf.size(),
  //                         bytes_transferred);

  //       std::cout << std::endl;

  //       sthis->do_read();
  //     });

  // serial_dev.async_read_some(
  //     boost::asio::buffer(rx_buf),
  //     [sthis](std::error_code error, size_t bytes_transferred) {
  //       if (error) {
  //         std::cout << "error reading data: " << error.message().c_str()
  //                   << std::endl;
  //         sthis->close();
  //         return;
  //       }
  //       std::cout << "data received: " << bytes_transferred << std::endl;
  //       sthis->parse_buff(sthis->rx_buf.data(), sthis->rx_buf.size(),
  //                         bytes_transferred);
  //       // for (auto el : sthis->rx_buf) {
  //       //   std::cout << el;
  //       // }
  //       // uint64_t cnt = 0;
  //       // for (int i = 7; i >= 0; i--) {
  //       //   uint8_t sh = 8 * i;
  //       //   printk(" tx_buf[%d] << %d: 0x%x \n", i, sh, tx_buf[i] << 8 * i);
  //       //   cnt = cnt | tx_buf[i] << 8 * (i);
  //       // }

  //       std::cout << std::endl;

  //       sthis->do_read();
  //     });
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

void IsyncSerial::parse_buff(uint8_t *buf, const size_t buffsize,
                             size_t bytes_recieved) {

  assert(buffsize >= bytes_recieved);

  for (; bytes_recieved > 0; bytes_recieved--) {
    auto el = *buf++;
    std::cout << std::setfill('0') << std::setw(2) << std::hex
              << (0xff & (uint8_t)el);
  }
  std::cout << '\n';
}

} // namespace ic_sync