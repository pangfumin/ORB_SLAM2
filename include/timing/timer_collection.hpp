// Copyright (c) 2015-2016, ETH Zurich, Wyss Zurich, Zurich Eye
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the ETH Zurich, Wyss Zurich, Zurich Eye nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL ETH Zurich, Wyss Zurich, Zurich Eye BE LIABLE
// FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <array>
#include <sstream>
#include <string>

//#include <ze/common/file_utils.hpp>

//#include <ze/common/string_utils.hpp>
#include "running_statistics.hpp"
#include "time_conversions.hpp"
#include "timer_statistics.hpp"
#include "types.hpp"

#include <glog/logging.h>

//! @file logging.hpp
//! Includes Glog framework and defines macros for DEBUG_CHECK_* which
//! can be compiled away.

#define DEBUG_CHECK(val) CHECK(val)
#define DEBUG_CHECK_NOTNULL(val) CHECK_NOTNULL(val)
#define DEBUG_CHECK_EQ(val1, val2) CHECK_EQ(val1, val2)
#define DEBUG_CHECK_NE(val1, val2) CHECK_NE(val1, val2)
#define DEBUG_CHECK_LE(val1, val2) CHECK_LE(val1, val2)
#define DEBUG_CHECK_LT(val1, val2) CHECK_LT(val1, val2)
#define DEBUG_CHECK_GE(val1, val2) CHECK_GE(val1, val2)
#define DEBUG_CHECK_GT(val1, val2) CHECK_GT(val1, val2)
#define DEBUG_CHECK_DOUBLE_EQ(val1, val2) CHECK_DOUBLE_EQ(val1, val2)
#define DEBUG_CHECK_NEAR(val1, val2, margin) CHECK_NEAR(val1, val2, margin)

namespace TimeStatistics {

/*! Collect statistics over multiple timings in milliseconds.
 *
 * Usage with explicit start and stop:
\code{.cpp}
  // Use the macro to declare the timer:
  DECLARE_TIMER(TimerName, timers, foo, bar);
  timers[TimerName::foo].start();
  ...
  timers[TimerName::foo].stop();
\endcode
 * Or RAII-style, use a TimedScope object that stops the timer when it
 * goes out of scope.
\code{.cpp}
  {
    auto t = timers[TimerName::foo].timeScope();
    ...
  }
\endcode
*/

inline std::string& leftTrimString(std::string& s) {
  s.erase(
      s.begin(),
      std::find_if(
          s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
  return s;
}

inline std::string& rightTrimString(std::string& s) {
  s.erase(
      std::find_if(
          s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace)))
          .base(),
      s.end());
  return s;
}
inline std::string& trimString(std::string& s) {
  return leftTrimString(rightTrimString(s));
}
inline std::vector<std::string> splitString(const std::string& s, char delim) {
  std::stringstream ss(s);
  std::string item;
  std::vector<std::string> items;
  while (std::getline(ss, item, delim)) {
    items.push_back(trimString(item));
  }
  return items;
}

template <typename TimerEnum>
class TimerCollection {
 public:
  using Timers =
      std::array<TimerStatistics, static_cast<uint32_t>(TimerEnum::dimension)>;
  using TimerNames = std::vector<std::string>;

  TimerCollection(){};

  //! This constructor is used by the macro DECLARE_TIMER() below.
  TimerCollection(const std::string& timer_names_comma_separated)
      : names_(splitString(timer_names_comma_separated, ',')) {
    CHECK_EQ(names_.size(), timers_.size());
  }

  TimerCollection(const std::vector<std::string>& timer_names)
      : names_(timer_names) {
    CHECK_EQ(names_.size(), timers_.size());
  }

  ~TimerCollection() = default;

  inline TimerStatistics& operator[](TimerEnum t) {
    return timers_[static_cast<uint32_t>(t)];
  }

  inline const TimerStatistics& operator[](TimerEnum t) const {
    return timers_[static_cast<uint32_t>(t)];
  }

  constexpr size_t size() const noexcept {
    return timers_.size();
  }

  //! Saves timings to file in YAML format.
  inline void saveToFile(
      const std::string& directory, const std::string& filename) const {
    /*
  std::ofstream fs;
  CHECK(isDir(directory));
  openOutputFileStream(joinPath(directory, filename), &fs);
  fs << *this;
     */
  }

  inline const Timers& timers() const {
    return timers_;
  }

  inline const TimerNames& names() const {
    return names_;
  }

  inline std::string print() {
    std::ostringstream out;
    out << *this;
    std::string str = out.str();
    return str;
  }

 private:
  Timers timers_;
  std::vector<std::string> names_;
};

//! Print Timer Collection:
template <typename TimerEnum>
std::ostream& operator<<(
    std::ostream& out, const TimerCollection<TimerEnum>& timers) {
  for (size_t i = 0u; i < timers.size(); ++i) {
    out << timers.names().at(i) << ":\n" << timers.timers().at(i).statistics();
  }
  return out;
}

template <typename TimerEnum>
std::string Print(const TimerCollection<TimerEnum>& timers) {
  std::ostringstream out;
  out << timers;
  std::string str = out.str();
  return str;
}

}  // end namespace ze

// http://en.cppreference.com/w/cpp/language/enum
#define DECLARE_TIMER(classname, membername, ...)             \
  enum class classname : uint32_t { __VA_ARGS__, dimension }; \
  TimeStatistics::TimerCollection<classname> membername {     \
    #__VA_ARGS__                                              \
  }
