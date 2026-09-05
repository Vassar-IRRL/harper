/**
 * Unit conversions and Protocol 2.0 control-table constants for X-series servos.
 *
 * Indirect Data layout (after configure maps Indirect Address → these fields):
 *   Write block @ 224 (4 B): Goal Position
 *   Read  block @ 228 (18 B): velocity, position, current/load, PWM,
 *                             moving, moving_status, hw_error, temp, voltage
 *
 * Control-loop I/O uses one GroupFastSyncRead (read block) and one
 * GroupSyncWrite (write block) per bus.
 *
 * Last modified: 2026-07-28
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "harper_control/dynamixel_types.hpp"

namespace harper_control {
namespace dynamixel_conversions {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kRevPerMinToRadPerSec = kTwoPi / 60.0;

// Protocol 2.0 X-series addresses shared across configured models.
constexpr uint16_t kAddrModelNumber = 0;
constexpr uint16_t kAddrBaudRate = 8;
constexpr uint16_t kAddrReturnDelayTime = 9;
constexpr uint16_t kAddrOperatingMode = 11;
constexpr uint16_t kAddrTorqueEnable = 64;
constexpr uint16_t kAddrHardwareErrorStatus = 70;
constexpr uint16_t kAddrGoalPosition = 116;
constexpr uint16_t kAddrMoving = 122;
constexpr uint16_t kAddrMovingStatus = 123;
constexpr uint16_t kAddrPresentPwm = 124;
constexpr uint16_t kAddrPresentCurrentOrLoad = 126;
constexpr uint16_t kAddrPresentVelocity = 128;
constexpr uint16_t kAddrPresentPosition = 132;
constexpr uint16_t kAddrPresentVoltage = 144;
constexpr uint16_t kAddrPresentTemperature = 146;
constexpr uint16_t kAddrProfileAcceleration = 108;
constexpr uint16_t kAddrProfileVelocity = 112;

// X-series Indirect Address / Data (XM/XL/XC family).
constexpr uint16_t kAddrIndirectAddress1 = 168;
constexpr uint16_t kAddrIndirectData1 = 224;

constexpr uint8_t kOperatingModePosition = 3;
constexpr uint8_t kBaudRateIndex4Mbps = 6; // EEPROM Baud Rate value for 4 Mbps

// Indirect write block: Goal Position only.
constexpr uint16_t kIndirectWriteStart = kAddrIndirectData1; // 224
constexpr uint16_t kIndirectWriteLen = 4;
constexpr uint16_t kIndirectWriteAddrIndex = 0; // Indirect Address slots 0..3

// Indirect read block immediately after the write block.
constexpr uint16_t kIndirectReadStart = kAddrIndirectData1 + kIndirectWriteLen; // 228
constexpr uint16_t kIndirectReadLen = 18;
constexpr uint16_t kIndirectReadAddrIndex = 4; // slots 4..21

// Offsets within the Indirect Read block (relative to kIndirectReadStart).
constexpr uint16_t kOffPresentVelocity = 0;
constexpr uint16_t kOffPresentPosition = 4;
constexpr uint16_t kOffPresentCurrentOrLoad = 8;
constexpr uint16_t kOffPresentPwm = 10;
constexpr uint16_t kOffMoving = 12;
constexpr uint16_t kOffMovingStatus = 13;
constexpr uint16_t kOffHardwareError = 14;
constexpr uint16_t kOffTemperature = 15;
constexpr uint16_t kOffVoltage = 16;

inline double pulses_to_radians(int32_t pulses, int resolution, int direction,
                                double zero_offset) {
    return static_cast<double>(direction) * static_cast<double>(pulses) *
               (kTwoPi / static_cast<double>(resolution)) +
           zero_offset;
}

inline int32_t radians_to_pulses(double radians, int resolution, int direction,
                                 double zero_offset) {
    const double scaled = static_cast<double>(direction) *
                          (radians - zero_offset) *
                          (static_cast<double>(resolution) / kTwoPi);
    return static_cast<int32_t>(std::llround(scaled));
}

inline double velocity_raw_to_rad_s(int32_t raw, double resolution_rev_per_min,
                                    int direction) {
    return static_cast<double>(direction) * static_cast<double>(raw) *
           resolution_rev_per_min * kRevPerMinToRadPerSec;
}

inline double compute_effort_nm(int32_t raw_source,
                                const EffortConversion& conversion) {
    switch (conversion.type) {
    case EffortConversionType::CurrentToNm:
        if (conversion.stall_current_a <= 0.0) {
            return 0.0;
        }
        return static_cast<double>(raw_source) * conversion.current_unit_a *
               (conversion.stall_torque_nm / conversion.stall_current_a);
    case EffortConversionType::LoadToNm:
        return (static_cast<double>(raw_source) * conversion.load_unit /
                100.0) *
               conversion.stall_torque_nm;
    case EffortConversionType::None:
    default:
        return 0.0;
    }
}

inline double clamp(double value, double lower, double upper) {
    return std::max(lower, std::min(value, upper));
}

} // namespace dynamixel_conversions
} // namespace harper_control
