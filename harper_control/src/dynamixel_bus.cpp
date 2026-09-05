/**
 * DynamixelBus — Indirect Address + Fast Sync Read/Write with I/O worker.
 *
 * Last modified: 2026-07-28
 */

#include "harper_control/dynamixel_bus.hpp"

#include <chrono>
#include <cmath>
#include <cstring>
#include <utility>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "harper_control/dynamixel_conversions.hpp"
#include "rclcpp/rclcpp.hpp"

#if __has_include("dynamixel_sdk/group_fast_sync_read.h")
#include "dynamixel_sdk/group_fast_sync_read.h"
#define HARPER_HAS_FAST_SYNC_READ 1
#elif __has_include("dynamixel_sdk/group_fast_sync_read.hpp")
#include "dynamixel_sdk/group_fast_sync_read.hpp"
#define HARPER_HAS_FAST_SYNC_READ 1
#else
#define HARPER_HAS_FAST_SYNC_READ 0
#endif

namespace harper_control {
namespace {

const char* kLoggerName = "DynamixelBus";

int32_t as_i32(uint32_t raw) {
    return static_cast<int32_t>(raw);
}

int16_t as_i16(uint32_t raw) {
    return static_cast<int16_t>(raw & 0xFFFF);
}

uint8_t as_u8(uint32_t raw) {
    return static_cast<uint8_t>(raw & 0xFF);
}

} // namespace

struct DynamixelBus::SyncReadHandle {
#if HARPER_HAS_FAST_SYNC_READ
    std::unique_ptr<dynamixel::GroupFastSyncRead> fast;
#endif
    std::unique_ptr<dynamixel::GroupSyncRead> normal;
    bool use_fast{false};

    bool addParam(uint8_t id) {
#if HARPER_HAS_FAST_SYNC_READ
        if (use_fast && fast) {
            return fast->addParam(id);
        }
#endif
        return normal && normal->addParam(id);
    }

    int txRxPacket() {
#if HARPER_HAS_FAST_SYNC_READ
        if (use_fast && fast) {
            return fast->txRxPacket();
        }
#endif
        return normal ? normal->txRxPacket() : COMM_TX_FAIL;
    }

    bool isAvailable(uint8_t id, uint16_t address, uint16_t len) const {
#if HARPER_HAS_FAST_SYNC_READ
        if (use_fast && fast) {
            return fast->isAvailable(id, address, len);
        }
#endif
        return normal && normal->isAvailable(id, address, len);
    }

    uint32_t getData(uint8_t id, uint16_t address, uint16_t len) const {
#if HARPER_HAS_FAST_SYNC_READ
        if (use_fast && fast) {
            return fast->getData(id, address, len);
        }
#endif
        return normal ? normal->getData(id, address, len) : 0;
    }
};

DynamixelBus::DynamixelBus() = default;

DynamixelBus::~DynamixelBus() {
    close();
}

bool DynamixelBus::open(const BusSettings& settings, std::string& error) {
    close();

    name_ = settings.name.empty() ? settings.port : settings.name;
    port_handler_ =
        dynamixel::PortHandler::getPortHandler(settings.port.c_str());
    packet_handler_ = dynamixel::PacketHandler::getPacketHandler(
        static_cast<float>(settings.protocol_version));
    if (port_handler_ == nullptr) {
        error =
            "Failed to create Dynamixel port handler for bus '" + name_ + "'";
        return false;
    }
    if (packet_handler_ == nullptr) {
        error =
            "Failed to create Dynamixel packet handler for bus '" + name_ + "'";
        return false;
    }

    if (!port_handler_->openPort()) {
        error = "Failed to open Dynamixel port '" + settings.port +
                "' for bus '" + name_ + "'";
        return false;
    }
    if (!port_handler_->setBaudRate(settings.baud_rate)) {
        error = "Failed to set Dynamixel baud rate " +
                std::to_string(settings.baud_rate) + " for bus '" + name_ +
                "' (servos must already match this baud, e.g. EEPROM index 6 "
                "for 4 Mbps)";
        port_handler_->closePort();
        return false;
    }

    open_ = true;
    RCLCPP_INFO(rclcpp::get_logger(kLoggerName),
                "Opened Dynamixel bus '%s' on %s @ %d baud (protocol %.1f)",
                name_.c_str(), settings.port.c_str(), settings.baud_rate,
                settings.protocol_version);
    return true;
}

void DynamixelBus::close() {
    stop_io_worker();
    sync_read_.reset();
    sync_write_goal_.reset();
    sync_groups_ready_ = false;
    worker_joints_.clear();

    if (port_handler_ != nullptr && open_) {
        port_handler_->closePort();
    }
    open_ = false;
    port_handler_ = nullptr;
    packet_handler_ = nullptr;
    name_.clear();
}

bool DynamixelBus::is_open() const {
    return open_;
}

const std::string& DynamixelBus::name() const {
    return name_;
}

bool DynamixelBus::belongs_to_bus(const JointStorage& storage) const {
    return storage.actuated && storage.bus.bus_name == name_;
}

bool DynamixelBus::check_comm(int result, uint8_t dxl_error,
                              const std::string& action,
                              std::string& error) const {
    if (result != COMM_SUCCESS) {
        error = action + ": " + packet_handler_->getTxRxResult(result);
        return false;
    }
    if (dxl_error != 0) {
        error = action + ": " + packet_handler_->getRxPacketError(dxl_error);
        return false;
    }
    return true;
}

bool DynamixelBus::read_u8(uint8_t id, uint16_t address, uint8_t& value,
                           std::string& error) const {
    uint8_t dxl_error = 0;
    const int result = packet_handler_->read1ByteTxRx(
        port_handler_, id, address, &value, &dxl_error);
    return check_comm(result, dxl_error, "read1Byte id=" + std::to_string(id),
                      error);
}

bool DynamixelBus::read_u16(uint8_t id, uint16_t address, uint16_t& value,
                            std::string& error) const {
    uint8_t dxl_error = 0;
    const int result = packet_handler_->read2ByteTxRx(
        port_handler_, id, address, &value, &dxl_error);
    return check_comm(result, dxl_error, "read2Byte id=" + std::to_string(id),
                      error);
}

bool DynamixelBus::write_u8(uint8_t id, uint16_t address, uint8_t value,
                            std::string& error) const {
    uint8_t dxl_error = 0;
    const int result = packet_handler_->write1ByteTxRx(
        port_handler_, id, address, value, &dxl_error);
    return check_comm(result, dxl_error, "write1Byte id=" + std::to_string(id),
                      error);
}

bool DynamixelBus::write_u16(uint8_t id, uint16_t address, uint16_t value,
                             std::string& error) const {
    uint8_t dxl_error = 0;
    const int result = packet_handler_->write2ByteTxRx(
        port_handler_, id, address, value, &dxl_error);
    return check_comm(result, dxl_error, "write2Byte id=" + std::to_string(id),
                      error);
}

bool DynamixelBus::write_u32(uint8_t id, uint16_t address, uint32_t value,
                             std::string& error) const {
    uint8_t dxl_error = 0;
    const int result = packet_handler_->write4ByteTxRx(
        port_handler_, id, address, value, &dxl_error);
    return check_comm(result, dxl_error, "write4Byte id=" + std::to_string(id),
                      error);
}

bool DynamixelBus::map_indirect_address(uint8_t id, uint16_t slot_index,
                                        uint16_t control_table_addr,
                                        std::string& error) {
    const uint16_t indirect_addr =
        static_cast<uint16_t>(dynamixel_conversions::kAddrIndirectAddress1 +
                              slot_index * 2);
    return write_u16(id, indirect_addr, control_table_addr, error);
}

bool DynamixelBus::setup_indirect_map_for_id(uint8_t id, std::string& error) {
    using namespace dynamixel_conversions;

    // Write block: Goal Position bytes → Indirect Data @ 224.
    for (uint16_t b = 0; b < 4; ++b) {
        if (!map_indirect_address(id, static_cast<uint16_t>(kIndirectWriteAddrIndex + b),
                                  static_cast<uint16_t>(kAddrGoalPosition + b),
                                  error)) {
            return false;
        }
    }

    // Read block: pack non-contiguous status into Indirect Data @ 228.
    const uint16_t sources[] = {
        static_cast<uint16_t>(kAddrPresentVelocity + 0),
        static_cast<uint16_t>(kAddrPresentVelocity + 1),
        static_cast<uint16_t>(kAddrPresentVelocity + 2),
        static_cast<uint16_t>(kAddrPresentVelocity + 3),
        static_cast<uint16_t>(kAddrPresentPosition + 0),
        static_cast<uint16_t>(kAddrPresentPosition + 1),
        static_cast<uint16_t>(kAddrPresentPosition + 2),
        static_cast<uint16_t>(kAddrPresentPosition + 3),
        static_cast<uint16_t>(kAddrPresentCurrentOrLoad + 0),
        static_cast<uint16_t>(kAddrPresentCurrentOrLoad + 1),
        static_cast<uint16_t>(kAddrPresentPwm + 0),
        static_cast<uint16_t>(kAddrPresentPwm + 1),
        kAddrMoving,
        kAddrMovingStatus,
        kAddrHardwareErrorStatus,
        kAddrPresentTemperature,
        static_cast<uint16_t>(kAddrPresentVoltage + 0),
        static_cast<uint16_t>(kAddrPresentVoltage + 1),
    };
    static_assert(sizeof(sources) / sizeof(sources[0]) == kIndirectReadLen,
                  "Indirect read source count must match read length");

    for (uint16_t i = 0; i < kIndirectReadLen; ++i) {
        if (!map_indirect_address(
                id, static_cast<uint16_t>(kIndirectReadAddrIndex + i),
                sources[i], error)) {
            return false;
        }
    }
    return true;
}

bool DynamixelBus::rebuild_sync_groups(
    const std::unordered_map<std::string, JointStorage>& joints,
    std::string& error) {
    if (!open_ || port_handler_ == nullptr || packet_handler_ == nullptr) {
        error = "Dynamixel bus is not open";
        return false;
    }

    sync_write_goal_ = std::make_unique<dynamixel::GroupSyncWrite>(
        port_handler_, packet_handler_,
        dynamixel_conversions::kIndirectWriteStart,
        dynamixel_conversions::kIndirectWriteLen);

    sync_read_ = std::make_unique<SyncReadHandle>();
#if HARPER_HAS_FAST_SYNC_READ
    sync_read_->use_fast = true;
    sync_read_->fast = std::make_unique<dynamixel::GroupFastSyncRead>(
        port_handler_, packet_handler_,
        dynamixel_conversions::kIndirectReadStart,
        dynamixel_conversions::kIndirectReadLen);
    RCLCPP_INFO(rclcpp::get_logger(kLoggerName),
                "Using GroupFastSyncRead on bus '%s'", name_.c_str());
#else
    sync_read_->use_fast = false;
    sync_read_->normal = std::make_unique<dynamixel::GroupSyncRead>(
        port_handler_, packet_handler_,
        dynamixel_conversions::kIndirectReadStart,
        dynamixel_conversions::kIndirectReadLen);
    RCLCPP_WARN(rclcpp::get_logger(kLoggerName),
                "GroupFastSyncRead unavailable; using GroupSyncRead on bus "
                "'%s'",
                name_.c_str());
#endif

    for (const auto& [joint_name, storage] : joints) {
        if (!belongs_to_bus(storage)) {
            continue;
        }
        const uint8_t id = static_cast<uint8_t>(storage.bus.id);
        if (!sync_read_->addParam(id)) {
            error = "Failed adding SyncRead param for '" + joint_name + "'";
            return false;
        }
    }

    sync_groups_ready_ = true;
    return true;
}

bool DynamixelBus::ping_and_verify(
    const std::unordered_map<std::string, JointStorage>& joints,
    std::string& error) {
    if (!open_) {
        error = "Dynamixel bus is not open";
        return false;
    }

    std::lock_guard<std::mutex> port_lock(port_mutex_);
    for (const auto& [joint_name, storage] : joints) {
        if (!belongs_to_bus(storage)) {
            continue;
        }

        const uint8_t id = static_cast<uint8_t>(storage.bus.id);
        uint8_t dxl_error = 0;
        const int result = packet_handler_->ping(port_handler_, id, &dxl_error);
        if (!check_comm(result, dxl_error, "ping joint '" + joint_name + "'",
                        error)) {
            return false;
        }

        uint16_t model_number = 0;
        if (!read_u16(id, dynamixel_conversions::kAddrModelNumber, model_number,
                      error)) {
            error = "Failed reading model number for '" + joint_name +
                    "': " + error;
            return false;
        }
        if (static_cast<int>(model_number) != storage.model.model_number) {
            error = "Model mismatch for '" + joint_name + "': expected " +
                    std::to_string(storage.model.model_number) + ", got " +
                    std::to_string(model_number);
            return false;
        }

        RCLCPP_INFO(rclcpp::get_logger(kLoggerName),
                    "Verified joint '%s' id=%u model=%s (%u)",
                    joint_name.c_str(), id, storage.bus.model.c_str(),
                    model_number);
    }

    return rebuild_sync_groups(joints, error);
}

bool DynamixelBus::configure_servo_tables(
    const std::unordered_map<std::string, JointStorage>& joints,
    std::string& error) {
    std::lock_guard<std::mutex> port_lock(port_mutex_);
    for (const auto& [joint_name, storage] : joints) {
        if (!belongs_to_bus(storage)) {
            continue;
        }
        const uint8_t id = static_cast<uint8_t>(storage.bus.id);

        // Torque off required for EEPROM writes.
        if (!write_u8(id, dynamixel_conversions::kAddrTorqueEnable, 0, error)) {
            error = "Failed disabling torque for '" + joint_name + "': " + error;
            return false;
        }
        if (!write_u8(id, dynamixel_conversions::kAddrReturnDelayTime, 0,
                      error)) {
            error = "Failed setting Return Delay Time=0 for '" + joint_name +
                    "': " + error;
            return false;
        }
        if (!setup_indirect_map_for_id(id, error)) {
            error = "Failed Indirect Address map for '" + joint_name +
                    "': " + error;
            return false;
        }
    }
    return true;
}

bool DynamixelBus::set_position_operating_mode(
    const std::unordered_map<std::string, JointStorage>& joints,
    std::string& error) {
    std::lock_guard<std::mutex> port_lock(port_mutex_);
    for (const auto& [joint_name, storage] : joints) {
        if (!belongs_to_bus(storage) || !storage.write_enabled) {
            continue;
        }
        const uint8_t id = static_cast<uint8_t>(storage.bus.id);
        if (!write_u8(id, dynamixel_conversions::kAddrTorqueEnable, 0, error)) {
            error = "Failed disabling torque before mode set for '" +
                    joint_name + "': " + error;
            return false;
        }
        if (!write_u8(id, dynamixel_conversions::kAddrOperatingMode,
                      dynamixel_conversions::kOperatingModePosition, error)) {
            error = "Failed setting position operating mode for '" +
                    joint_name + "': " + error;
            return false;
        }
    }
    return true;
}

void DynamixelBus::decode_indirect_read(
    uint8_t id, const JointStorage& storage,
    std::unordered_map<std::string, double>& out_states) const {
    using namespace dynamixel_conversions;
    if (!sync_read_) {
        return;
    }

    auto get_u32 = [&](uint16_t abs_addr, uint16_t len) -> uint32_t {
        return sync_read_->getData(id, abs_addr, len);
    };
    auto available = [&](uint16_t abs_addr, uint16_t len) -> bool {
        return sync_read_->isAvailable(id, abs_addr, len);
    };

    const uint16_t base = kIndirectReadStart;
    if (!available(base + kOffPresentVelocity, 4) ||
        !available(base + kOffPresentPosition, 4)) {
        return;
    }

    const int32_t velocity_raw =
        as_i32(get_u32(base + kOffPresentVelocity, 4));
    const int32_t position_raw =
        as_i32(get_u32(base + kOffPresentPosition, 4));

    out_states[hardware_interface::HW_IF_POSITION] = pulses_to_radians(
        position_raw, storage.model.position_resolution, storage.bus.direction,
        storage.bus.zero_offset);
    out_states[hardware_interface::HW_IF_VELOCITY] = velocity_raw_to_rad_s(
        velocity_raw, storage.model.velocity_resolution, storage.bus.direction);

    if (available(base + kOffHardwareError, 1)) {
        out_states["dxl_hardware_error_status"] =
            static_cast<double>(as_u8(get_u32(base + kOffHardwareError, 1)));
    }
    if (available(base + kOffMoving, 1) &&
        storage.model.telemetry.count("dxl_moving") &&
        storage.model.telemetry.at("dxl_moving")) {
        out_states["dxl_moving"] =
            static_cast<double>(as_u8(get_u32(base + kOffMoving, 1)));
    }
    if (available(base + kOffMovingStatus, 1) &&
        storage.model.telemetry.count("dxl_moving_status") &&
        storage.model.telemetry.at("dxl_moving_status")) {
        out_states["dxl_moving_status"] =
            static_cast<double>(as_u8(get_u32(base + kOffMovingStatus, 1)));
    }
    if (available(base + kOffPresentPwm, 2) &&
        storage.model.telemetry.count("dxl_pwm") &&
        storage.model.telemetry.at("dxl_pwm")) {
        out_states["dxl_pwm"] =
            static_cast<double>(as_i16(get_u32(base + kOffPresentPwm, 2)));
    }

    if (available(base + kOffPresentCurrentOrLoad, 2)) {
        const int16_t current_or_load =
            as_i16(get_u32(base + kOffPresentCurrentOrLoad, 2));
        if (storage.model.telemetry.count("dxl_current") &&
            storage.model.telemetry.at("dxl_current")) {
            out_states["dxl_current"] = static_cast<double>(current_or_load);
            if (storage.model.effort.type ==
                EffortConversionType::CurrentToNm) {
                out_states[hardware_interface::HW_IF_EFFORT] =
                    compute_effort_nm(current_or_load, storage.model.effort);
            }
        }
        if (storage.model.telemetry.count("dxl_load") &&
            storage.model.telemetry.at("dxl_load")) {
            out_states["dxl_load"] = static_cast<double>(current_or_load);
            if (storage.model.effort.type == EffortConversionType::LoadToNm) {
                out_states[hardware_interface::HW_IF_EFFORT] =
                    compute_effort_nm(current_or_load, storage.model.effort);
            }
        }
    }

    if (available(base + kOffTemperature, 1) &&
        storage.model.telemetry.count("dxl_temperature") &&
        storage.model.telemetry.at("dxl_temperature")) {
        out_states["dxl_temperature"] =
            static_cast<double>(as_u8(get_u32(base + kOffTemperature, 1)));
    }
    if (available(base + kOffVoltage, 2) &&
        storage.model.telemetry.count("dxl_voltage") &&
        storage.model.telemetry.at("dxl_voltage")) {
        const uint16_t voltage =
            static_cast<uint16_t>(get_u32(base + kOffVoltage, 2) & 0xFFFF);
        out_states["dxl_voltage"] = 0.1 * static_cast<double>(voltage);
    }
}

bool DynamixelBus::cycle_io(std::string& error) {
    using namespace dynamixel_conversions;
    std::lock_guard<std::mutex> port_lock(port_mutex_);
    if (!sync_groups_ready_) {
        error = "Sync groups not ready";
        return false;
    }

    BusCommandFrame commands;
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        commands = command_frame_;
    }

    if (commands.allow_writes && sync_write_goal_) {
        sync_write_goal_->clearParam();
        bool any = false;
        for (const auto& [joint_name, storage] : worker_joints_) {
            if (!belongs_to_bus(storage) || !storage.write_enabled) {
                continue;
            }
            const auto cmd_it = commands.position_commands.find(joint_name);
            if (cmd_it == commands.position_commands.end() ||
                !std::isfinite(cmd_it->second)) {
                continue;
            }

            double command = cmd_it->second;
            if (storage.bus.clamp_commands) {
                command = clamp(command, storage.bus.position_min,
                                storage.bus.position_max);
            } else if (command < storage.bus.position_min ||
                       command > storage.bus.position_max) {
                error = "Command out of range for '" + joint_name + "'";
                return false;
            }

            const int32_t pulses = radians_to_pulses(
                command, storage.model.position_resolution,
                storage.bus.direction, storage.bus.zero_offset);
            uint8_t param[4] = {
                static_cast<uint8_t>(pulses & 0xFF),
                static_cast<uint8_t>((pulses >> 8) & 0xFF),
                static_cast<uint8_t>((pulses >> 16) & 0xFF),
                static_cast<uint8_t>((pulses >> 24) & 0xFF),
            };
            if (!sync_write_goal_->addParam(static_cast<uint8_t>(storage.bus.id),
                                            param)) {
                error = "Failed adding SyncWrite for '" + joint_name + "'";
                return false;
            }
            any = true;
        }
        if (any) {
            const int wr = sync_write_goal_->txPacket();
            sync_write_goal_->clearParam();
            if (wr != COMM_SUCCESS) {
                error = "GroupSyncWrite on bus '" + name_ + "': " +
                        packet_handler_->getTxRxResult(wr);
                return false;
            }
        }
    }

    int rd = COMM_TX_FAIL;
    if (sync_read_) {
        rd = sync_read_->txRxPacket();
    }
    if (rd != COMM_SUCCESS) {
        error = "GroupFast/SyncRead on bus '" + name_ + "': " +
                packet_handler_->getTxRxResult(rd);
        return false;
    }

    BusStateFrame frame;
    frame.ok = true;
    for (const auto& [joint_name, storage] : worker_joints_) {
        if (!belongs_to_bus(storage)) {
            continue;
        }
        std::unordered_map<std::string, double> states;
        decode_indirect_read(static_cast<uint8_t>(storage.bus.id), storage,
                             states);
        if (states.count("dxl_hardware_error_status") &&
            states.at("dxl_hardware_error_status") != 0.0) {
            frame.hardware_fault = true;
        }
        frame.states.emplace(joint_name, std::move(states));
    }

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state_frame_ = std::move(frame);
    }
    return true;
}

bool DynamixelBus::read_states_once(
    std::unordered_map<std::string, JointStorage>& joints,
    std::string& error) {
    worker_joints_ = joints;
    BusCommandFrame idle;
    idle.allow_writes = false;
    publish_commands(idle);
    if (!cycle_io(error)) {
        return false;
    }
    const BusStateFrame frame = latest_states();
    if (!frame.ok) {
        error = frame.error.empty() ? "read_states_once failed" : frame.error;
        return false;
    }
    for (auto& [joint_name, storage] : joints) {
        const auto it = frame.states.find(joint_name);
        if (it == frame.states.end()) {
            continue;
        }
        for (const auto& [iface, value] : it->second) {
            storage.states[iface] = value;
        }
    }
    return true;
}

bool DynamixelBus::write_profiles(
    const std::unordered_map<std::string, JointStorage>& joints,
    std::string& error) {
    std::lock_guard<std::mutex> port_lock(port_mutex_);
    for (const auto& [joint_name, storage] : joints) {
        if (!belongs_to_bus(storage) || !storage.write_enabled) {
            continue;
        }
        const uint8_t id = static_cast<uint8_t>(storage.bus.id);
        if (!write_u32(id,
                       static_cast<uint16_t>(
                           storage.model.profile_acceleration_address),
                       static_cast<uint32_t>(storage.bus.profile_acceleration),
                       error) ||
            !write_u32(
                id,
                static_cast<uint16_t>(storage.model.profile_velocity_address),
                static_cast<uint32_t>(storage.bus.profile_velocity), error)) {
            error = "Failed writing profile for '" + joint_name + "': " + error;
            return false;
        }
    }
    return true;
}

bool DynamixelBus::set_torque(
    const std::unordered_map<std::string, JointStorage>& joints, bool enable,
    std::string& error) {
    std::lock_guard<std::mutex> port_lock(port_mutex_);
    for (const auto& [joint_name, storage] : joints) {
        if (!belongs_to_bus(storage) || !storage.write_enabled) {
            continue;
        }
        const uint8_t id = static_cast<uint8_t>(storage.bus.id);
        if (!write_u8(id, dynamixel_conversions::kAddrTorqueEnable,
                      enable ? 1 : 0, error)) {
            error = "Failed setting torque for '" + joint_name + "': " + error;
            return false;
        }
    }
    return true;
}

void DynamixelBus::publish_commands(const BusCommandFrame& commands) {
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        command_frame_ = commands;
    }
    command_cv_.notify_one();
}

BusStateFrame DynamixelBus::latest_states() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return state_frame_;
}

bool DynamixelBus::start_io_worker(
    const std::unordered_map<std::string, JointStorage>& joints,
    std::string& error) {
    if (!open_ || !sync_groups_ready_) {
        error = "Cannot start I/O worker before configure";
        return false;
    }
    stop_io_worker();
    worker_joints_ = joints;
    io_running_ = true;
    io_thread_ = std::thread([this]() { io_thread_main(); });
    RCLCPP_INFO(rclcpp::get_logger(kLoggerName),
                "Started I/O worker thread for bus '%s'", name_.c_str());
    return true;
}

void DynamixelBus::stop_io_worker() {
    if (!io_running_ && !io_thread_.joinable()) {
        return;
    }
    io_running_ = false;
    command_cv_.notify_all();
    if (io_thread_.joinable()) {
        io_thread_.join();
    }
}

void DynamixelBus::io_thread_main() {
    while (io_running_) {
        {
            std::unique_lock<std::mutex> lock(command_mutex_);
            command_cv_.wait_for(lock, std::chrono::milliseconds(5),
                                 [this]() { return !io_running_; });
        }
        if (!io_running_) {
            break;
        }

        std::string error;
        if (!cycle_io(error)) {
            BusStateFrame frame;
            frame.ok = false;
            frame.error = error;
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                state_frame_ = std::move(frame);
            }
            static auto last_log = std::chrono::steady_clock::now() -
                                   std::chrono::seconds(2);
            const auto now = std::chrono::steady_clock::now();
            if (now - last_log > std::chrono::seconds(1)) {
                RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s",
                             error.c_str());
                last_log = now;
            }
        }
    }
}

} // namespace harper_control
