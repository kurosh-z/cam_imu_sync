#pragma once
#ifndef ICAM_SYNC_TIMESTAMPBUFF_UTILS_HPP_
#define ICAM_SYNC_TIMESTAMPBUFF_UTILS_HPP_
#include <ros/ros.h>

namespace ic_sync {
namespace utils {

template <typename _Tp, std::size_t _Nm> class TimestampBuff {

protected:
  std::array<_Tp, _Nm> ts_buff; // actual imu reading timestamp sent from driver
  std::array<ros::Time, _Nm> received_ts_buff; // ros time when timestamp is
                                               // received from serial
  std::array<ros::Time, 2> triggered_buff;
  size_t tr_idx = 0;
  size_t _N = _Nm;
  size_t _head = 0;
  size_t _rhead = 0;
  const float ZERO_TIME_OFFSET_US =
      1100; // estimated time of sending 100 bytes of data over serial with
            // 921600 baudrate ref:
            // https://lucidar.me/en/serialib/most-used-baud-rates-table/
            // 	10.851 µs/byte * 100 bytes = 1085.1 us ~ 1100 us
  ros::Time zero_time;
  ros::Time calculated_ts;
  ros::Time last_calculated_ts;
  bool zero_set = false;
  bool first_ts_set = false;

  uint64_t last_seq;
  uint64_t curr_seq;

  double time_sec = 0;
  double last_time_sec = 0;

public:
  void set_zero_time(ros::Time ts) {

    if (zero_set) {
      ROS_WARN("zero time already set");
      return;
    }
    zero_time.fromNSec(ts.toNSec() - ZERO_TIME_OFFSET_US * 1e3);
    calculated_ts = zero_time;
    zero_set = true;
  }

  bool is_zero_time_set() { return zero_set; }
  size_t get_head() { return _head; }
  size_t index() {
    return (double)_head - 1 < 0 ? (double)_head - 1 + _N : (double)_head - 1;
  }
  size_t last_index() {
    return (double)_head - 2 < 0 ? (double)_head - 2 + _N : (double)_head - 2;
  }

  _Tp current_timestamp() { return ts_buff.at(index()); }
  _Tp last_timestamp() { return ts_buff.at(last_index()); }
  _Tp at(size_t idx) { return ts_buff.at(idx); }

  size_t size() { return _N; }

  double append_triggered_ts(ros::Time ts) {
    double diff = -1;
    if (triggered_buff.empty()) {
      diff = 0;
    }
    triggered_buff.at(tr_idx) = ts;
    tr_idx = (tr_idx + 1) % 2;
    if (diff == 0) {
      return 0.0;
    } else {
      return (triggered_buff.at((tr_idx + 1) % 2) - triggered_buff.at(tr_idx))
                 .toSec() *
             1000;
    }
  }

  /****
   * @brief append received ros::Time timestamp when the message received from
   serial
   * @param ts ros::Time timestamp to append
   * @return void
   * @note this function should be called before append_ts()
           used for debugging purpose
  */
  void append_received_ts(ros::Time ts) {
    received_ts_buff.at(_rhead) = ts;
    _rhead = (_rhead + 1) % _N;
  }
  /***
   * @brief append timestamp to timestamp buffer
   * @param ts timestamp [us] to append
   * @param seq sequence number of the timestamp
   * @param triggered if the timestamp is triggered or not
   *  @return std::pair<ros::Time, bool> first= ros::Time timestamp calculated
   * from the drivers timestamp , second= bool if there is an error
   */
  std::pair<ros::Time, bool> append_ts(_Tp ts, uint64_t seq, bool triggered) {
    if (!zero_set) {
      ROS_ERROR("zero time not set");
      return std::make_pair(ros::Time(0), true);
    }
    try {
      // don't append and don't update curr_seq if seq is not correct
      if ((curr_seq > seq)) {
        ROS_ERROR_STREAM("last_seq: " << curr_seq << " > curr_seq: " << seq);
        return std::make_pair(ros::Time(0), true);
      }
      // we didn't update the _head yet so current_timestamp() gives the last
      // one
      auto dt = double(ts - current_timestamp());
      if (dt < 0) {
        ROS_ERROR_STREAM("Error calculating the timestamp- dt[us]:  " << dt);
        return std::make_pair(ros::Time(0), true);
      }

      last_seq = curr_seq;
      curr_seq = seq;

      // ROS_INFO_STREAM("seq: " << seq);
      if (curr_seq - last_seq > 1) {
        ROS_WARN_STREAM("missed some sequences! seq_diff: "
                        << (int64_t)(curr_seq - last_seq));
      }
      ts_buff[_head] = ts;
      _head = (_head + 1) % _N;

      // there should be at leas one timestamp in the buffer to calculate the
      // ros::Time timestamp from the imu timestamp

      if (!first_ts_set) {
        first_ts_set = true;
        return std::make_pair(calculated_ts, false);
      }
      // last_time_sec = time_sec;
      // time_sec = last_time_sec + dt * 1e-6; // dt[us] * 1e-6 -> dt[sec]

      calculated_ts.fromSec(calculated_ts.toSec() +
                            dt * 1e-6); // convert to sec

      // std::cout << "-------------------------------\n";
      // ROS_INFO_STREAM("[append_ts] seq: "
      //                 << seq << " dt[ms]: " << dt / 1e3 << " time_dt [ms]: "
      //                 << (calculated_ts - last_calculated_ts).toSec() * 1e3);
      last_calculated_ts = calculated_ts;
      return std::make_pair(calculated_ts, false);
    } catch (const std::exception &e) {
      ROS_ERROR_STREAM("Error calculating the timestamp: " << e.what());
      return std::make_pair(ros::Time(0), true);
    }

    // return ros::Time::fromNSec()
    // for debugging
    // if (!ts_buff.empty()) {
    //   // calculate_diff();
    //   try {

    //     double ros_diff =
    //         (received_ts_buff.at(index()) -
    //         received_ts_buff.at(last_index())).toSec() * 1e6;
    //     double ts_diff = ts_buff.at(index()) - ts_buff.at(last_index());
    //     std::cout << "seq: " << seq << " tr: " << triggered
    //               << " --------------------------\n";
    //     ROS_INFO_STREAM("ros diff: " << ros_diff << " ts diff: " << ts_diff
    //                                  << "  dif: " << ros_diff - ts_diff);
    //   } catch (const std::exception &e) {
    //     std::cerr << e.what() << '\n';
    //   }
    // }
  }

  uint64_t get_last_seq() { return last_seq; }
}; // namespace utils

} // namespace utils
} // namespace ic_sync

#endif