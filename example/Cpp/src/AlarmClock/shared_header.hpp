/*
* This is a minimal example of an alarmclock implemeted using the 
* features lingua franca supplies.
* 
* This is just an extract and simplification from the main project
* which you can find here: https://github.com/revol-xut/lf-alarm-clock
*
* Author: Tassilo Tanneberer <tassilo.tanneberger@tu-dresden.de>
*/

#ifndef SHARED_HEADER_INCLUDE_GUARD
#define SHARED_HEADER_INCLUDE_GUARD

#include <iostream>
#include <string>
#include <thread>
#include <fstream>
#include <filesystem>

struct Event {
    std::string message_;
    long time_stamp_;
};

constexpr const char* kMusicDir = "~/music/AlarmClock/";
constexpr const char* kFile = "./alarm_clock_events.csv";
constexpr const char* kPlayerCommand = "mpg321";
constexpr const char* kKillCommand = "kill";
constexpr const char* kPidofCommand = "pidof";

constexpr unsigned short kPort = 8680;

#endif //SHARED_HEADER_INCLUDE_GUARD

