/**
 * DynamixelBus — one U2D2 / serial port with a dedicated I/O worker thread.
 *
 * Configure-time (controller thread): open, ping, Return Delay=0, Indirect
 * Address map, operating mode, profiles, torque.
 *
 * Run-time (worker thread): GroupSyncWrite goal block + GroupFastSyncRead
 * state block through Indirect Data, then publish into a double-buffered
 * state frame. The controller_manager thread only swaps command/state frames.
 *
 * Last modified: 2026-07-28
 */

#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

#include "harper_control/dynamixel_frames.hpp"
#include "harper_control/dynamixel_types.hpp"

namespace dynamixel {
class PortHandler;
class PacketHandler;
class GroupSyncWrite;
} // namespace dynamixel

namespace harper_control {

class DynamixelBus {
  public:
    DynamixelBus();
    ~DynamixelBus();

    DynamixelBus(const DynamixelBus&) = delete;
    DynamixelBus& operator=(const DynamixelBus&) = delete;

    bool open(const BusSettings& settings, std::string& error);
    void close();
    bool is_open() const;
    const std::string& name() const;

    bool
    ping_and_verify(const std::unordered_map<std::string, JointStorage>& joints,
                    std::string& error);

    bool configure_servo_tables(
        const std::unordered_map<std::string, JointStorage>& joints,
        std::string& error);

    bool set_position_operating_mode(
        const std::unordered_map<std::string, JointStorage>& joints,
        std::string& error);

    bool
    write_profiles(const std::unordered_map<std::string, JointStorage>& joints,
                   std::string& error);

    bool set_torque(const std::unordered_map<std::string, JointStorage>& joints,
                    bool enable, std::string& error);

    bool read_states_once(
        std::unordered_map<std::string, JointStorage>& joints,
        std::string& error);

    bool start_io_worker(
        const std::unordered_map<std::string, JointStorage>& joints,
        std::string& error);
    void stop_io_worker();

    void publish_commands(const BusCommandFrame& commands);
    BusStateFrame latest_states() const;

  private:
    struct SyncReadHandle; // hides GroupFastSyncRead vs GroupSyncRead

    bool read_u8(uint8_t id, uint16_t address, uint8_t& value,
                 std::string& error) const;
    bool read_u16(uint8_t id, uint16_t address, uint16_t& value,
                  std::string& error) const;
    bool write_u8(uint8_t id, uint16_t address, uint8_t value,
                  std::string& error) const;
    bool write_u16(uint8_t id, uint16_t address, uint16_t value,
                   std::string& error) const;
    bool write_u32(uint8_t id, uint16_t address, uint32_t value,
                   std::string& error) const;
    bool check_comm(int result, uint8_t dxl_error, const std::string& action,
                    std::string& error) const;
    bool belongs_to_bus(const JointStorage& storage) const;
    bool map_indirect_address(uint8_t id, uint16_t slot_index,
                              uint16_t control_table_addr, std::string& error);
    bool setup_indirect_map_for_id(uint8_t id, std::string& error);
    bool rebuild_sync_groups(
        const std::unordered_map<std::string, JointStorage>& joints,
        std::string& error);
    bool cycle_io(std::string& error);
    void io_thread_main();
    void decode_indirect_read(
        uint8_t id, const JointStorage& model_joint,
        std::unordered_map<std::string, double>& out_states) const;

    std::string name_;
    dynamixel::PortHandler* port_handler_{nullptr};
    dynamixel::PacketHandler* packet_handler_{nullptr};
    std::unique_ptr<SyncReadHandle> sync_read_;
    std::unique_ptr<dynamixel::GroupSyncWrite> sync_write_goal_;
    bool sync_groups_ready_{false};
    bool open_{false};

    // Serializes PortHandler access between the I/O worker and lifecycle calls
    // (torque / profiles / Indirect Address setup).
    mutable std::mutex port_mutex_;

    std::unordered_map<std::string, JointStorage> worker_joints_;

    std::thread io_thread_;
    std::atomic<bool> io_running_{false};
    std::mutex command_mutex_;
    std::condition_variable command_cv_;
    BusCommandFrame command_frame_;
    mutable std::mutex state_mutex_;
    BusStateFrame state_frame_;
};

} // namespace harper_control
