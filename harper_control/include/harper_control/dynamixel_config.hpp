/**
*
* This file contains the functions to load the Dynamixel configuration from the YAML file.
* It also binds the joints to the configuration.
*
* Last modified: 2026-07-27
*
*/


#pragma once

#include <string>
#include <unordered_map>

#include "harper_control/dynamixel_types.hpp"

namespace harper_control {

struct LoadedDynamixelConfig {
    SharedDynamixelSettings shared;
    std::unordered_map<std::string, BusSettings> buses;
    std::unordered_map<std::string, ModelMetadata> models;
    std::unordered_map<std::string, JointConfig> joints;
};

std::string resolve_config_path(const std::unordered_map<std::string, std::string>& hardware_params,
                                const std::string& key, const std::string& default_filename);

bool load_dynamixel_config(const std::string& bus_config_path,
                           const std::string& models_config_path, LoadedDynamixelConfig& config,
                           std::string& error);

bool bind_joints_to_config(std::unordered_map<std::string, JointStorage>& joints,
                           const LoadedDynamixelConfig& config, std::string& error);

} // namespace harper_control
