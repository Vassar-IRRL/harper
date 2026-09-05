/**
 * HarperDynamixelSystem — dual-arm Dynamixel SystemInterface for ros2_control.
 *
 * See harper_dynamixel_system.cpp for lifecycle and I/O-worker notes.
 *
 * Last modified: 2026-07-28
 */

#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "harper_control/dynamixel_bus.hpp"
#include "harper_control/dynamixel_types.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace harper_control {

class HarperDynamixelSystem : public hardware_interface::SystemInterface {
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(HarperDynamixelSystem)

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::
        LifecycleNodeInterface::CallbackReturn;

    CallbackReturn
    on_init(const hardware_interface::HardwareInfo& info) override;
    CallbackReturn
    on_configure(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn
    on_activate(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn
    on_error(const rclcpp_lifecycle::State& previous_state) override;

    std::vector<hardware_interface::StateInterface>
    export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface>
    export_command_interfaces() override;

    hardware_interface::return_type
    read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    hardware_interface::return_type
    write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  private:
    void apply_metadata_interfaces();
    void add_state_interface(const std::string& joint_name,
                             const std::string& interface_name);
    void copy_mimic_states();
    void apply_bus_states_to_joints();
    void publish_commands_to_buses();
    void deactivate_storage();
    void close_buses();
    bool buses_open() const;
    bool disable_torque_safe();

    std::unordered_map<std::string, JointStorage> joints_;
    std::vector<std::pair<std::string, std::string>> state_interface_order_;
    std::vector<std::pair<std::string, std::string>> command_interface_order_;
    std::string bus_config_path_;
    std::string models_config_path_;
    SharedDynamixelSettings shared_;
    std::unordered_map<std::string, BusSettings> bus_settings_;
    std::unordered_map<std::string, std::unique_ptr<DynamixelBus>> buses_;
    bool active_{false};
};

} // namespace harper_control
