/**
 * Per-bus command / state frames exchanged between the controller_manager
 * thread and long-lived Dynamixel I/O worker threads.
 *
 * Last modified: 2026-07-28
 */

#pragma once

#include <string>
#include <unordered_map>

namespace harper_control {

struct BusCommandFrame {
    // joint name -> position command (rad). NaN / missing = skip.
    std::unordered_map<std::string, double> position_commands;
    bool allow_writes{false};
};

struct BusStateFrame {
    // joint name -> interface name -> value
    std::unordered_map<std::string, std::unordered_map<std::string, double>>
        states;
    bool ok{true};
    bool hardware_fault{false};
    std::string error;
};

} // namespace harper_control
