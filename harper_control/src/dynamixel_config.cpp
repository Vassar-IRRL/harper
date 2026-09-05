/**
 *
 * This file contains the implementation of the DynamixelConfig class, which is
 * used to parse the Dynamixel config file and return the config as a
 * LoadedDynamixelConfig object.
 *
 * Last modified: 2026-07-28
 *
 */

#include "harper_control/dynamixel_config.hpp"

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "yaml-cpp/yaml.h"

namespace harper_control {
namespace {

/**
 * True when a YAML node is present and holds a real scalar (not null / ~).
 * Treats the string "None" as absent for backwards compatibility.
 */
bool has_numeric_scalar(const YAML::Node& node) {
    if (!node || node.IsNull() || !node.IsScalar()) {
        return false;
    }
    const auto text = node.as<std::string>();
    return !text.empty() && text != "None" && text != "null" && text != "~";
}

double as_double_or(const YAML::Node& node, double fallback) {
    if (!has_numeric_scalar(node)) {
        return fallback;
    }
    return node.as<double>();
}

/**
 * Parses the effort conversion from the YAML node and returns an
 * EffortConversion object.
 */
EffortConversion parse_effort_conversion(const YAML::Node& node) {
    EffortConversion conversion;
    if (!node || node.IsNull()) {
        return conversion;
    }

    const auto type = node["type"] ? node["type"].as<std::string>() : "";
    if (type == "current_to_nm") {
        conversion.type = EffortConversionType::CurrentToNm;
        conversion.present_current_address = node["present_current"].as<int>();
        conversion.current_unit_a = node["current_unit_a"].as<double>();
        conversion.stall_torque_nm = node["stall_torque_nm"].as<double>();
        conversion.stall_current_a = node["stall_current_a"].as<double>();
    } else if (type == "load_to_nm") {
        conversion.type = EffortConversionType::LoadToNm;
        conversion.present_load_address = node["present_load"].as<int>();
        conversion.load_unit = node["load_unit"].as<double>();
        conversion.stall_torque_nm = node["stall_torque_nm"].as<double>();
    }
    return conversion;
}

/**
 * Parses the model metadata from the YAML node and returns a ModelMetadata
 * object.
 */
ModelMetadata parse_model_metadata(const std::string& key,
                                   const YAML::Node& node) {
    ModelMetadata metadata;
    metadata.model_name =
        node["model_name"] ? node["model_name"].as<std::string>() : key;
    metadata.model_number = node["model_number"].as<int>();
    metadata.protocol_version = node["protocol_version"].as<double>();

    const auto position = node["position"];
    metadata.present_position = position["present_position"].as<int>();
    metadata.goal_position = position["goal_position"].as<int>();
    metadata.position_resolution = position["resolution"].as<int>();

    const auto velocity = node["velocity"];
    metadata.present_velocity = velocity["present_velocity"].as<int>();
    metadata.velocity_resolution = velocity["resolution"].as<double>();

    if (node["profile"]) {
        const auto profile = node["profile"];
        metadata.profile_acceleration_address =
            profile["profile_acceleration"].as<int>();
        metadata.profile_velocity_address =
            profile["profile_velocity"].as<int>();
        if (profile["velocity_unit"]) {
            metadata.profile_velocity_unit =
                profile["velocity_unit"].as<double>();
        }
        if (profile["acceleration_unit"]) {
            metadata.profile_acceleration_unit =
                profile["acceleration_unit"].as<double>();
        }
    }

    if (node["telemetry"]) {
        for (const auto& name : telemetry_interface_names()) {
            if (node["telemetry"][name]) {
                metadata.telemetry[name] = node["telemetry"][name].as<bool>();
            }
        }
    }

    metadata.effort = parse_effort_conversion(node["effort_conversion"]);
    return metadata;
}

/**
 * Parses the joint config from the YAML node and returns a JointConfig object.
 */
JointConfig parse_joint_config(const YAML::Node& node,
                               const std::string& bus_name,
                               const SharedDynamixelSettings& shared) {
    JointConfig config;
    config.bus_name = bus_name;
    config.id = node["id"].as<int>();
    config.model = node["model"].as<std::string>();
    config.direction = node["direction"] ? node["direction"].as<int>() : 1;
    config.zero_offset =
        node["zero_offset"] ? node["zero_offset"].as<double>() : 0.0;
    config.profile_velocity = node["profile_velocity"]
                                  ? node["profile_velocity"].as<int>()
                                  : shared.default_profile_velocity;
    config.profile_acceleration = node["profile_acceleration"]
                                      ? node["profile_acceleration"].as<int>()
                                      : shared.default_profile_acceleration;
    config.position_min =
        as_double_or(node["position_min"], shared.default_position_min);
    config.position_max =
        as_double_or(node["position_max"], shared.default_position_max);
    config.clamp_commands = shared.clamp_commands;
    if (node["clamp_commands"]) {
        config.clamp_commands = node["clamp_commands"].as<bool>();
    }
    return config;
}

/**
 * Resolves the enabled joints from the config and returns a set of joint names.
 */
std::unordered_set<std::string>
resolve_enabled_joints(const LoadedDynamixelConfig& config) {
    std::unordered_set<std::string> enabled;
    for (const auto& group_name : config.shared.enabled_joint_groups) {
        if (group_name == "all") {
            for (const auto& [joint_name, _] : config.joints) {
                (void)_;
                enabled.insert(joint_name);
            }
            continue;
        }

        const auto group_it = config.shared.joint_groups.find(group_name);
        if (group_it != config.shared.joint_groups.end()) {
            enabled.insert(group_it->second.begin(), group_it->second.end());
            continue;
        }

        if (config.joints.count(group_name) > 0) {
            enabled.insert(group_name);
        }
    }
    return enabled;
}

/**
 * Parses the shared settings from the YAML node and returns a
 * SharedDynamixelSettings object.
 */
bool parse_shared_settings(const YAML::Node& root,
                           SharedDynamixelSettings& shared,
                           std::string& error) {
    shared.readonly = root["readonly"] ? root["readonly"].as<bool>() : true;
    shared.clamp_commands = true;
    if (root["limit_policy"] && root["limit_policy"]["clamp"]) {
        shared.clamp_commands = root["limit_policy"]["clamp"].as<bool>();
    }

    if (root["defaults"]) {
        const auto defaults = root["defaults"];
        shared.default_profile_velocity =
            defaults["profile_velocity"]
                ? defaults["profile_velocity"].as<int>()
                : 0;
        shared.default_profile_acceleration =
            defaults["profile_acceleration"]
                ? defaults["profile_acceleration"].as<int>()
                : 0;
        if (has_numeric_scalar(defaults["position_min"])) {
            shared.default_position_min = defaults["position_min"].as<double>();
        }
        if (has_numeric_scalar(defaults["position_max"])) {
            shared.default_position_max = defaults["position_max"].as<double>();
        }
    }

    if (root["enabled_joint_groups"]) {
        for (const auto& item : root["enabled_joint_groups"]) {
            shared.enabled_joint_groups.push_back(item.as<std::string>());
        }
    }

    if (root["joint_groups"]) {
        for (const auto& item : root["joint_groups"]) {
            std::vector<std::string> joints;
            for (const auto& joint : item.second) {
                joints.push_back(joint.as<std::string>());
            }
            shared.joint_groups.emplace(item.first.as<std::string>(), joints);
        }
    }

    return true;
}

} // namespace

/**
 * Resolves the config path from the hardware parameters and returns the config
 * path.
 */
std::string resolve_config_path(
    const std::unordered_map<std::string, std::string>& hardware_params,
    const std::string& key, const std::string& default_filename) {
    const auto it = hardware_params.find(key);
    if (it != hardware_params.end() && !it->second.empty()) {
        return it->second;
    }
    return ament_index_cpp::get_package_share_directory("harper_control") +
           "/" + default_filename;
}

/**
 * Loads the Dynamixel config from the given paths and returns a
 * LoadedDynamixelConfig object.
 */
bool load_dynamixel_config(const std::string& bus_config_path,
                           const std::string& models_config_path,
                           LoadedDynamixelConfig& config, std::string& error) {
    try {
        const YAML::Node bus_root = YAML::LoadFile(bus_config_path);
        const YAML::Node models_root = YAML::LoadFile(models_config_path);
        const YAML::Node root = bus_root["dynamixel"];
        const YAML::Node models = models_root["dynamixel_models"];

        if (!root || !models) {
            error = "Missing dynamixel or dynamixel_models root keys";
            return false;
        }

        config = LoadedDynamixelConfig{};
        if (!parse_shared_settings(root, config.shared, error)) {
            return false;
        }

        for (const auto& item : models) {
            const auto key = item.first.as<std::string>();
            config.models.emplace(key, parse_model_metadata(key, item.second));
        }

        if (!root["buses"] || !root["buses"].IsMap() ||
            root["buses"].size() == 0) {
            error = "dynamixel.buses must be a non-empty map of named buses";
            return false;
        }

        for (const auto& bus_item : root["buses"]) {
            const auto bus_name = bus_item.first.as<std::string>();
            const YAML::Node bus_node = bus_item.second;

            BusSettings settings;
            settings.name = bus_name;
            settings.port = bus_node["port"]
                                ? bus_node["port"].as<std::string>()
                                : "/dev/ttyUSB0";
            settings.baud_rate = bus_node["baud_rate"]
                                     ? bus_node["baud_rate"].as<int>()
                                     : 115200;
            settings.protocol_version =
                bus_node["protocol_version"]
                    ? bus_node["protocol_version"].as<double>()
                    : 2.0;

            const YAML::Node joint_nodes = bus_node["joints"];
            if (!joint_nodes || !joint_nodes.IsMap() ||
                joint_nodes.size() == 0) {
                error = "Bus '" + bus_name + "' is missing joints";
                return false;
            }

            for (const auto& joint_item : joint_nodes) {
                const auto joint_name = joint_item.first.as<std::string>();
                if (config.joints.count(joint_name) > 0) {
                    error = "Joint '" + joint_name +
                            "' is assigned to more than one bus";
                    return false;
                }
                config.joints.emplace(
                    joint_name, parse_joint_config(joint_item.second, bus_name,
                                                   config.shared));
            }

            config.buses.emplace(bus_name, settings);
        }
    } catch (const std::exception& ex) {
        error = std::string("Failed to load Dynamixel config: ") + ex.what();
        return false;
    }

    return true;
}

/**
 * Binds the joints to the config and returns true if the joints are found in
 * the config.
 */
bool bind_joints_to_config(
    std::unordered_map<std::string, JointStorage>& joints,
    const LoadedDynamixelConfig& config, std::string& error) {
    const auto enabled = resolve_enabled_joints(config);

    for (auto& [joint_name, storage] : joints) {
        if (storage.mimic) {
            continue;
        }

        const auto joint_it = config.joints.find(joint_name);
        if (joint_it == config.joints.end()) {
            continue;
        }

        const auto model_it = config.models.find(joint_it->second.model);
        if (model_it == config.models.end()) {
            error = "No model metadata for '" + joint_it->second.model +
                    "' (joint '" + joint_name + "')";
            return false;
        }

        if (config.buses.count(joint_it->second.bus_name) == 0) {
            error = "Joint '" + joint_name + "' references unknown bus '" +
                    joint_it->second.bus_name + "'";
            return false;
        }

        storage.actuated = true;
        storage.bus = joint_it->second;
        storage.model = model_it->second;
        storage.write_enabled = enabled.count(joint_name) > 0;
    }

    return true;
}

} // namespace harper_control
