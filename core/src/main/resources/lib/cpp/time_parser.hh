/*
 * Copyright (c) 2020, TU Dresden.

 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:

 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.

 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include "reactor-cpp/reactor-cpp.hh"
#include <sstream>

std::stringstream &operator>>(std::stringstream& in, reactor::Duration& dur);
#include "CLI/cxxopts.hpp"

#include <fstream>
#include <iostream>
#include <string>
#include <regex>
#include <algorithm>

bool iequals(const std::string& a, const std::string& b)
{
    return std::equal(a.begin(), a.end(),
                      b.begin(), b.end(),
                      [](char a, char b) {
                          return tolower(a) == tolower(b);
                      });
}

class argument_incorrect_type_with_reason : public cxxopts::OptionParseException
{
 public:
  explicit argument_incorrect_type_with_reason(
    const std::string& arg,
    const std::string& reason)
  : cxxopts::OptionParseException(
      "Argument " + cxxopts::LQUOTE + arg + cxxopts::RQUOTE + " failed to parse (" + reason + ")"
    )
  {}
};


std::string validate_time_string(const std::string& time);

/**
 * converts a reactor::Duration to a string with ns as unit
 */
std::string time_to_string(const reactor::Duration& dur) {
  if (dur == reactor::Duration::max()) {
    return "forever";
  }

  std::stringstream ss;
  ss << dur.count() << " ns";
  return ss.str();
}

template<typename T>
std::string any_to_string(const T val){
    std::stringstream ss;
    ss << val;
    return ss.str();
}

std::stringstream &operator>>(std::stringstream& in, reactor::Duration& dur) {
  double value;
  std::string unit;

  const std::string validation_msg = validate_time_string(in.str());
  if (!validation_msg.empty()) {
    // throw cxxopts error
    throw argument_incorrect_type_with_reason(in.str(), validation_msg);
  }

  if (iequals(in.str(), "forever")) {
    in >> unit; // parse the entire string
    dur = reactor::Duration::max();
    return in;
  }

  // try to read as double
  in >> value;

  if (value == 0.0) {
    dur = reactor::Duration::zero();
    if (!in.eof()) {
      // parse whatever remains
      in >> unit;
    }
  } else {
    in >> unit;
    if (unit == "nsec" || unit == "nsecs" || unit == "ns" ) {
      std::chrono::duration<double, std::nano> tmp{value};
      dur = std::chrono::duration_cast<reactor::Duration>(tmp);
    } else if (unit == "usec" || unit == "usecs" || unit == "us") {
      std::chrono::duration<double, std::micro> tmp{value};
      dur = std::chrono::duration_cast<reactor::Duration>(tmp);
    } else if (unit == "msec" || unit == "msecs" || unit == "ms") {
      std::chrono::duration<double, std::milli> tmp{value};
      dur = std::chrono::duration_cast<reactor::Duration>(tmp);
    } else if (unit == "sec" || unit == "secs" ||
               unit == "second" || unit == "seconds" || unit == "s") {
      std::chrono::duration<double, std::ratio<1, 1>> tmp{value};
      dur = std::chrono::duration_cast<reactor::Duration>(tmp);
    } else if (unit == "min" || unit == "mins" ||
               unit == "minute" || unit == "minutes" || unit == "m") {
      std::chrono::duration<double, std::ratio<60, 1>> tmp{value};
      dur = std::chrono::duration_cast<reactor::Duration>(tmp);
    } else if (unit == "hour" || unit == "hours" || unit == "h") {
      std::chrono::duration<double, std::ratio<3600, 1>> tmp{value};
      dur = std::chrono::duration_cast<reactor::Duration>(tmp);
    } else if (unit == "day" || unit == "days" || unit == "d") {
      std::chrono::duration<double, std::ratio<24*3600, 1>> tmp{value};
      dur = std::chrono::duration_cast<reactor::Duration>(tmp);
    } else if (unit == "week" || unit == "weeks" || unit == "w") {
      std::chrono::duration<double, std::ratio<7*24*3600, 1>> tmp{value};
      dur = std::chrono::duration_cast<reactor::Duration>(tmp);
    } else {
      // mark as error
      in.setstate(std::ifstream::failbit);
    }
  }

  return in;
}

/**
*   Tests for correct syntax in unit usage for time strings
**/
std::string validate_time_string(const std::string& time) {
  auto trimmed = std::regex_replace(time, std::regex("^ +| +$|( ) +"), "$1");
  if (trimmed.size() == 0) {
    return "The empty string is not a valid time!";
  } else if (trimmed[0] == '-') {
    return "Negative values are not a valid time!";
  } else if (iequals("forever", time)) {
    return ""; // "forever" is a valid value
  } else if (trimmed.find_first_not_of("0.") == std::string::npos) {
    return "";
  } else {
    auto pos = trimmed.find_first_not_of("0123456789. \n\r\t");
    if (pos == std::string::npos) {
      return "No unit given!";
    } else {
      auto unit = trimmed.substr(pos);
      if (unit == "nsec" || unit == "nsecs" || unit == "ns" ||
          unit == "usec" || unit == "usecs" || unit == "us" ||
          unit == "msec" || unit == "msecs" || unit == "ms" ||
          unit == "sec" || unit == "secs" ||
          unit == "second" || unit == "seconds" || unit == "s" ||
          unit == "min" || unit == "mins" ||
          unit == "minute" || unit == "minutes" || unit == "m" ||
          unit == "hour" || unit == "hours" || unit == "h" ||
          unit == "day" || unit == "days" || unit == "d" ||
          unit == "week" || unit == "weeks" || unit == "w") {
        return "";
      } else {
        std::stringstream ss;
        ss << "Not a valid unit: " << unit;
        return ss.str();
      }
    }
  }
  return "Unexpected error!";
}


