#pragma once
#ifndef ICAM_SYNC_THREAD_UTILS_HPP_
#define ICAM_SYNC_THREAD_UTILS_HPP_

#include <pthread.h>

#include <cstdio>
#include <sstream>
#include <string>
#include <thread>
#include <utility>

namespace ic_sync {
namespace utils {

/**
 * @brief Make printf-formatted std::string
 *
 */
template <typename... Args>
std::string format(const std::string &fmt, Args... args) {
  // C++11 specify that string store elements continously
  std::string ret;

  auto sz = std::snprintf(nullptr, 0, fmt.c_str(), args...);
  ret.reserve(sz + 1);
  ret.resize(sz); // to be sure there have room for \0
  std::snprintf(&ret.front(), ret.capacity() + 1, fmt.c_str(), args...);
  return ret;
}

/**
 * @brief Set name to current thread, printf-like
 * @param[in] name name for thread
 * @return true if success
 *
 * @note Only for Linux target
 * @todo add for other posix system
 */
template <typename... Args>
bool set_this_thread_name(const std::string &name, Args &&... args) {
  auto new_name = format(name, std::forward<Args>(args)...);

#ifdef __APPLE__
  return pthread_setname_np(new_name.c_str()) == 0;
#else
  pthread_t pth = pthread_self();
  return pthread_setname_np(pth, new_name.c_str()) == 0;
#endif
}

/**
 * @brief Convert to string objects with operator <<
 */
template <typename T> inline const std::string to_string_ss(T &obj) {
  std::ostringstream ss;
  ss << obj;
  return ss.str();
}

constexpr size_t operator"" _KiB(unsigned long long sz) // NOLINT
{
  return sz * 1024;
}

} // namespace utils
} // namespace ic_sync

#endif