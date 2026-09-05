/**
 * Shared Dynamixel / ros2_control types for HARPER.
 *
 * JointStorage holds per-joint state and command buffers that
 * HarperDynamixelSystem exports as hardware_interface handles.
 * Actuated joints bind to a bus ID + model; mimic joints only
 * copy state from a lead joint (no bus I/O).
 *
 * Last modified: 2026-07-28
 */

#pragma once

#include <string>
#include <unordered_map>
#include <vector>

namespace harper_control {

enum class EffortConversionType {
    None,
    CurrentToNm,
    LoadToNm,
};

struct EffortConversion {
    EffortConversionType type{EffortConversionType::None};
    int present_current_address{0};
    int present_load_address{0};
    double current_unit_a{0.0};
    double load_unit{0.0};
    double stall_torque_nm{0.0};
    double stall_current_a{0.0};
};

struct ModelMetadata {
    std::string model_name;
    int model_number{0};
    double protocol_version{2.0};
    int present_position{132};
    int goal_position{116};
    int position_resolution{4096};
    int present_velocity{128};
    double velocity_resolution{0.229};
    int profile_acceleration_address{108};
    int profile_velocity_address{112};
    double profile_velocity_unit{0.229};
    double profile_acceleration_unit{214.577};
    std::unordered_map<std::string, bool> telemetry;
    EffortConversion effort;
};

struct JointConfig {
    std::string bus_name;
    int id{0};
    std::string model;
    // direction: +1 or -1 maps encoder sense to URDF joint axis.
    // zero_offset: radians added after direction scaling (sim2real calibration).
    int direction{1};
    double zero_offset{0.0};
    int profile_velocity{0};
    int profile_acceleration{0};
    double position_min{-3.141592653589793};
    double position_max{3.141592653589793};
    bool clamp_commands{true};
};

struct JointStorage {
    bool mimic{false};
    std::string mimic_lead; // lead joint name when mimic==true
    bool actuated{false};
    bool write_enabled{false};
    JointConfig bus;
    ModelMetadata model;
    std::unordered_map<std::string, double> states;
    std::unordered_map<std::string, double> commands;
};

struct BusSettings {
    std::string name;
    std::string port{"/dev/ttyUSB0"};
    int baud_rate{115200};
    double protocol_version{2.0};
};

struct SharedDynamixelSettings {
    bool readonly{true};
    bool clamp_commands{true};
    int default_profile_velocity{0};
    int default_profile_acceleration{0};
    double default_position_min{-3.141592653589793};
    double default_position_max{3.141592653589793};
    std::vector<std::string> enabled_joint_groups;
    std::unordered_map<std::string, std::vector<std::string>> joint_groups;
};

inline const std::vector<std::string>& telemetry_interface_names() {
    static const std::vector<std::string> names = {
        "dxl_current",       "dxl_pwm",
        "dxl_load",          "dxl_voltage",
        "dxl_temperature",   "dxl_moving",
        "dxl_moving_status", "dxl_hardware_error_status",
    };
    return names;
}

} // namespace harper_control
