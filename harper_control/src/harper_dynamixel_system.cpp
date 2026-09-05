/**
 * HarperDynamixelSystem — dual-arm Dynamixel SystemInterface for ros2_control.
 *
 * Lifecycle (Humble):
 *   on_init      — parse URDF + YAML (no port I/O)
 *   on_configure — open buses, ping, Return Delay=0, Indirect Address map,
 *                  start long-lived I/O worker threads
 *   on_activate  — position mode + profiles + torque (unless readonly)
 *   read/write   — only swap double-buffered frames with worker threads
 *   on_error     — disable torque
 *
 * Each bus owns one worker thread doing GroupSyncWrite + GroupFastSyncRead
 * through Indirect Data. Left/right workers run concurrently without
 * spawning std::async every control cycle.
 *
 * Last modified: 2026-07-28
 */

#include "harper_control/harper_dynamixel_system.hpp"

#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "harper_control/dynamixel_config.hpp"
#include "harper_control/dynamixel_frames.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace harper_control {
namespace {

const char* kLoggerName = "HarperDynamixelSystem";

bool parse_bool_param(const std::unordered_map<std::string, std::string>& params,
                      const std::string& key, bool fallback) {
    const auto it = params.find(key);
    if (it == params.end() || it->second.empty()) {
        return fallback;
    }
    return it->second == "true";
}

} // namespace

HarperDynamixelSystem::CallbackReturn
HarperDynamixelSystem::on_init(const hardware_interface::HardwareInfo& info) {
    if (hardware_interface::SystemInterface::on_init(info) !=
        CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    joints_.clear();
    state_interface_order_.clear();
    command_interface_order_.clear();
    close_buses();
    bus_settings_.clear();

    bus_config_path_ = resolve_config_path(info_.hardware_parameters, "bus_config",
                                           "config/dynamixel_bus.yaml");
    models_config_path_ = resolve_config_path(
        info_.hardware_parameters, "models_config", "config/dynamixel_models.yaml");

    for (const auto& joint : info_.joints) {
        auto& storage = joints_[joint.name];
        const auto mimic_it = joint.parameters.find("mimic");
        storage.mimic = mimic_it != joint.parameters.end();
        if (storage.mimic) {
            storage.mimic_lead = mimic_it->second;
        }

        for (const auto& state_interface : joint.state_interfaces) {
            add_state_interface(joint.name, state_interface.name);
        }

        for (const auto& command_interface : joint.command_interfaces) {
            if (storage.mimic) {
                RCLCPP_ERROR(rclcpp::get_logger(kLoggerName),
                             "Mimic joint '%s' must not export command interfaces",
                             joint.name.c_str());
                return CallbackReturn::ERROR;
            }
            storage.commands.emplace(command_interface.name,
                                     std::numeric_limits<double>::quiet_NaN());
            command_interface_order_.emplace_back(joint.name,
                                                  command_interface.name);
        }
    }

    LoadedDynamixelConfig loaded;
    std::string error;
    if (!load_dynamixel_config(bus_config_path_, models_config_path_, loaded,
                               error) ||
        !bind_joints_to_config(joints_, loaded, error)) {
        RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s", error.c_str());
        return CallbackReturn::ERROR;
    }

    shared_ = loaded.shared;
    bus_settings_ = loaded.buses;
    shared_.readonly =
        parse_bool_param(info_.hardware_parameters, "readonly", shared_.readonly);
    apply_metadata_interfaces();

    size_t write_count = 0;
    for (const auto& [_, storage] : joints_) {
        (void)_;
        if (storage.write_enabled) {
            ++write_count;
        }
    }

    std::string bus_summary;
    for (const auto& [name, settings] : bus_settings_) {
        if (!bus_summary.empty()) {
            bus_summary += ", ";
        }
        bus_summary +=
            name + "=" + settings.port + "@" + std::to_string(settings.baud_rate);
    }

    RCLCPP_INFO(rclcpp::get_logger(kLoggerName),
                "Configured %zu joints from %s (readonly=%s, write_enabled=%zu, "
                "buses=[%s])",
                joints_.size(), bus_config_path_.c_str(),
                shared_.readonly ? "true" : "false", write_count,
                bus_summary.c_str());

    return CallbackReturn::SUCCESS;
}

HarperDynamixelSystem::CallbackReturn
HarperDynamixelSystem::on_configure(const rclcpp_lifecycle::State&) {
    deactivate_storage();
    close_buses();

    for (const auto& [name, settings] : bus_settings_) {
        auto bus = std::make_unique<DynamixelBus>();
        std::string error;
        if (!bus->open(settings, error)) {
            RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s", error.c_str());
            close_buses();
            return CallbackReturn::ERROR;
        }
        buses_.emplace(name, std::move(bus));
    }

    std::string error;
    for (auto& [name, bus] : buses_) {
        (void)name;
        if (!bus->ping_and_verify(joints_, error) ||
            !bus->configure_servo_tables(joints_, error)) {
            RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s", error.c_str());
            close_buses();
            return CallbackReturn::ERROR;
        }
    }

    if (!shared_.readonly) {
        for (auto& [name, bus] : buses_) {
            (void)name;
            if (!bus->set_position_operating_mode(joints_, error)) {
                RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s",
                             error.c_str());
                close_buses();
                return CallbackReturn::ERROR;
            }
        }
    }

    for (auto& [name, bus] : buses_) {
        (void)name;
        if (!bus->read_states_once(joints_, error)) {
            RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s", error.c_str());
            close_buses();
            return CallbackReturn::ERROR;
        }
    }
    copy_mimic_states();

    for (auto& [name, bus] : buses_) {
        (void)name;
        if (!bus->start_io_worker(joints_, error)) {
            RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s", error.c_str());
            close_buses();
            return CallbackReturn::ERROR;
        }
    }

    RCLCPP_INFO(rclcpp::get_logger(kLoggerName),
                "Configured %zu Dynamixel bus(es) with I/O workers",
                buses_.size());
    return CallbackReturn::SUCCESS;
}

HarperDynamixelSystem::CallbackReturn
HarperDynamixelSystem::on_cleanup(const rclcpp_lifecycle::State&) {
    disable_torque_safe();
    deactivate_storage();
    close_buses();
    return CallbackReturn::SUCCESS;
}

HarperDynamixelSystem::CallbackReturn
HarperDynamixelSystem::on_activate(const rclcpp_lifecycle::State&) {
    if (!buses_open()) {
        RCLCPP_ERROR(rclcpp::get_logger(kLoggerName),
                     "Cannot activate without open Dynamixel buses");
        return CallbackReturn::ERROR;
    }

    // Pull latest worker state, then seed commands from measured positions.
    apply_bus_states_to_joints();
    copy_mimic_states();

    for (auto& [joint_name, storage] : joints_) {
        (void)joint_name;
        const auto position =
            storage.states.find(hardware_interface::HW_IF_POSITION);
        for (auto& [interface_name, command] : storage.commands) {
            if (interface_name == hardware_interface::HW_IF_POSITION &&
                position != storage.states.end()) {
                command = position->second;
            }
        }
    }

    if (!shared_.readonly) {
        std::string error;
        for (auto& [name, bus] : buses_) {
            (void)name;
            if (!bus->write_profiles(joints_, error) ||
                !bus->set_torque(joints_, true, error)) {
                RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s",
                             error.c_str());
                return CallbackReturn::ERROR;
            }
        }
        RCLCPP_WARN(rclcpp::get_logger(kLoggerName),
                    "Torque enabled for write-enabled joints (readonly=false)");
    } else {
        RCLCPP_INFO(rclcpp::get_logger(kLoggerName),
                    "Activated in readonly mode (no torque/writes)");
    }

    active_ = true;
    publish_commands_to_buses();
    return CallbackReturn::SUCCESS;
}

HarperDynamixelSystem::CallbackReturn
HarperDynamixelSystem::on_deactivate(const rclcpp_lifecycle::State&) {
    disable_torque_safe();
    deactivate_storage();
    return CallbackReturn::SUCCESS;
}

HarperDynamixelSystem::CallbackReturn
HarperDynamixelSystem::on_shutdown(const rclcpp_lifecycle::State&) {
    disable_torque_safe();
    deactivate_storage();
    close_buses();
    return CallbackReturn::SUCCESS;
}

HarperDynamixelSystem::CallbackReturn
HarperDynamixelSystem::on_error(const rclcpp_lifecycle::State&) {
    disable_torque_safe();
    deactivate_storage();
    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
HarperDynamixelSystem::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> interfaces;
    interfaces.reserve(state_interface_order_.size());
    for (const auto& [joint_name, interface_name] : state_interface_order_) {
        interfaces.emplace_back(joint_name, interface_name,
                                &joints_.at(joint_name).states.at(interface_name));
    }
    return interfaces;
}

std::vector<hardware_interface::CommandInterface>
HarperDynamixelSystem::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> interfaces;
    interfaces.reserve(command_interface_order_.size());
    for (const auto& [joint_name, interface_name] : command_interface_order_) {
        interfaces.emplace_back(
            joint_name, interface_name,
            &joints_.at(joint_name).commands.at(interface_name));
    }
    return interfaces;
}

hardware_interface::return_type
HarperDynamixelSystem::read(const rclcpp::Time&, const rclcpp::Duration&) {
    if (!buses_open()) {
        return hardware_interface::return_type::OK;
    }

    bool fault = false;
    std::string error;
    for (auto& [name, bus] : buses_) {
        (void)name;
        const BusStateFrame frame = bus->latest_states();
        if (!frame.ok) {
            error = frame.error.empty() ? "bus I/O failed" : frame.error;
            fault = true;
            break;
        }
        if (frame.hardware_fault) {
            error = "Dynamixel hardware fault on bus '" + bus->name() + "'";
            fault = true;
            break;
        }
        for (const auto& [joint_name, states] : frame.states) {
            auto joint_it = joints_.find(joint_name);
            if (joint_it == joints_.end()) {
                continue;
            }
            for (const auto& [iface, value] : states) {
                if (joint_it->second.states.count(iface)) {
                    joint_it->second.states[iface] = value;
                }
            }
        }
    }

    if (fault) {
        RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s", error.c_str());
        disable_torque_safe();
        return hardware_interface::return_type::ERROR;
    }

    copy_mimic_states();
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type
HarperDynamixelSystem::write(const rclcpp::Time&, const rclcpp::Duration&) {
    if (!active_ || shared_.readonly || !buses_open()) {
        return hardware_interface::return_type::OK;
    }

    for (const auto& [joint_name, storage] : joints_) {
        const auto position_command =
            storage.commands.find(hardware_interface::HW_IF_POSITION);
        if (position_command == storage.commands.end()) {
            continue;
        }
        if (!std::isfinite(position_command->second)) {
            RCLCPP_ERROR(rclcpp::get_logger(kLoggerName),
                         "Rejecting non-finite position command for joint '%s'",
                         joint_name.c_str());
            disable_torque_safe();
            return hardware_interface::return_type::ERROR;
        }
    }

    publish_commands_to_buses();
    return hardware_interface::return_type::OK;
}

void HarperDynamixelSystem::apply_metadata_interfaces() {
    for (auto& [joint_name, storage] : joints_) {
        if (!storage.actuated) {
            continue;
        }
        for (const auto& name : telemetry_interface_names()) {
            const auto it = storage.model.telemetry.find(name);
            if (it != storage.model.telemetry.end() && it->second) {
                add_state_interface(joint_name, name);
            }
        }
        if (storage.model.effort.type != EffortConversionType::None) {
            add_state_interface(joint_name, hardware_interface::HW_IF_EFFORT);
        }
        RCLCPP_INFO(rclcpp::get_logger(kLoggerName),
                    "Joint '%s': bus=%s id=%d model=%s write_enabled=%s "
                    "profile_v=%d profile_a=%d",
                    joint_name.c_str(), storage.bus.bus_name.c_str(),
                    storage.bus.id, storage.bus.model.c_str(),
                    storage.write_enabled ? "true" : "false",
                    storage.bus.profile_velocity,
                    storage.bus.profile_acceleration);
    }
}

void HarperDynamixelSystem::add_state_interface(
    const std::string& joint_name, const std::string& interface_name) {
    auto& storage = joints_.at(joint_name);
    if (storage.states.find(interface_name) != storage.states.end()) {
        return;
    }
    storage.states.emplace(interface_name, 0.0);
    state_interface_order_.emplace_back(joint_name, interface_name);
}

void HarperDynamixelSystem::copy_mimic_states() {
    for (auto& [joint_name, storage] : joints_) {
        if (!storage.mimic || storage.mimic_lead.empty()) {
            continue;
        }
        const auto lead_it = joints_.find(storage.mimic_lead);
        if (lead_it == joints_.end()) {
            continue;
        }
        const auto& lead = lead_it->second;
        for (const char* iface :
             {hardware_interface::HW_IF_POSITION,
              hardware_interface::HW_IF_VELOCITY}) {
            const auto src = lead.states.find(iface);
            auto dst = storage.states.find(iface);
            if (src == lead.states.end() || dst == storage.states.end()) {
                continue;
            }
            dst->second = src->second;
        }
        (void)joint_name;
    }
}

void HarperDynamixelSystem::apply_bus_states_to_joints() {
    for (auto& [name, bus] : buses_) {
        (void)name;
        const BusStateFrame frame = bus->latest_states();
        for (const auto& [joint_name, states] : frame.states) {
            auto joint_it = joints_.find(joint_name);
            if (joint_it == joints_.end()) {
                continue;
            }
            for (const auto& [iface, value] : states) {
                if (joint_it->second.states.count(iface)) {
                    joint_it->second.states[iface] = value;
                }
            }
        }
    }
}

void HarperDynamixelSystem::publish_commands_to_buses() {
    for (auto& [name, bus] : buses_) {
        (void)name;
        BusCommandFrame frame;
        frame.allow_writes = active_ && !shared_.readonly;
        for (const auto& [joint_name, storage] : joints_) {
            if (!storage.actuated || storage.bus.bus_name != bus->name()) {
                continue;
            }
            const auto cmd =
                storage.commands.find(hardware_interface::HW_IF_POSITION);
            if (cmd != storage.commands.end()) {
                frame.position_commands.emplace(joint_name, cmd->second);
            }
        }
        bus->publish_commands(frame);
    }
}

void HarperDynamixelSystem::deactivate_storage() {
    active_ = false;
    for (auto& [joint_name, storage] : joints_) {
        (void)joint_name;
        for (auto& [interface_name, command] : storage.commands) {
            (void)interface_name;
            command = std::numeric_limits<double>::quiet_NaN();
        }
    }
    // Tell workers to stop writing goals.
    if (buses_open()) {
        for (auto& [name, bus] : buses_) {
            (void)name;
            BusCommandFrame idle;
            idle.allow_writes = false;
            bus->publish_commands(idle);
        }
    }
}

void HarperDynamixelSystem::close_buses() {
    for (auto& [_, bus] : buses_) {
        (void)_;
        if (bus) {
            bus->close();
        }
    }
    buses_.clear();
}

bool HarperDynamixelSystem::buses_open() const {
    if (buses_.empty() || buses_.size() != bus_settings_.size()) {
        return false;
    }
    for (const auto& [_, bus] : buses_) {
        (void)_;
        if (!bus || !bus->is_open()) {
            return false;
        }
    }
    return true;
}

bool HarperDynamixelSystem::disable_torque_safe() {
    if (!buses_open() || shared_.readonly) {
        return true;
    }
    std::string error;
    for (auto& [name, bus] : buses_) {
        (void)name;
        if (!bus->set_torque(joints_, false, error)) {
            RCLCPP_ERROR(rclcpp::get_logger(kLoggerName),
                         "Torque disable failed: %s", error.c_str());
            return false;
        }
    }
    return true;
}

} // namespace harper_control

PLUGINLIB_EXPORT_CLASS(harper_control::HarperDynamixelSystem,
                       hardware_interface::SystemInterface)
