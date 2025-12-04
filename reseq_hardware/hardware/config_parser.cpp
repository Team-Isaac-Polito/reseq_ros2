#include "reseq_hardware/config_parser.hpp"
#include <stdint.h>                         // for uint8_t
#include <yaml-cpp/yaml.h>                  // for Node, LoadFile, ...
#include <rclcpp/logger.hpp>                // for get_logger
#include <rclcpp/logging.hpp>               // for RCLCPP_INFO
#include <stdexcept>                        // for runtime_error
#include <vector>                           // for vector
#include "reseq_hardware/canbus.hpp"        // for CanID
#include "reseq_hardware/reseq.hpp"         // for PayloadFieldMapping, CanM...

namespace reseq_hardware
{

/**
 * @brief Generates a unique joint name string by combining module ID and joint name.
 * @param mod_id Module identifier.
 * @param joint_name Name of the joint.
 * @return Combined string in the format "modX__joint_name" (X is decimal).
 */
std::string mod_joint(uint8_t mod_id, const std::string & joint_name)
{
  return "mod" + std::to_string(mod_id) + "__" + joint_name;
}

/**
 * @brief Parses a mapping type string to the corresponding MappingType enum.
 * @param type_str String representing the mapping type.
 * @return Corresponding MappingType value.
 * @throws std::runtime_error if the type is unknown.
 */
MappingType parse_type(const std::string & type_str)
{
  if (type_str == "state") {
    return MappingType::JOINT_STATE;
  } else if (type_str == "command") {
    return MappingType::JOINT_COMMAND;
  } else if (type_str == "telemetry") {
    return MappingType::TOPIC_TELEMETRY;
  } else {
    throw std::runtime_error("Unknown mapping type: " + type_str);
  }
}

std::map<CanID, CanMessageMapping> parse_can_config_file(
  const std::string & filename,
  int num_modules, int mk_version, 
  std::vector<std::pair<std::string, std::string>> & topic_mappings)
{
  std::map<CanID, CanMessageMapping> can_mappings;
  YAML::Node config = YAML::LoadFile(filename);
  for (const auto & mapping : config) {
    // Determine which modules this mapping applies to
    std::string modules = mapping["modules"].as<std::string>("all");
    for (int i = 1; i <= num_modules; ++i) {
      if (modules == "skip_first" && i == 1) {
        continue;
      }
      if (modules == "only_first" && i != 1) {
        continue;
      }

      // Parse Message mapping for this module
      CanMessageMapping msg_mapping;
      msg_mapping.id.mod_id = idx_to_mod(i, mk_version);
      msg_mapping.id.msg_id = mapping["msg_id"].as<uint8_t>();
      msg_mapping.length = mapping["length"].as<uint8_t>(4);
      msg_mapping.is_command = mapping["is_command"].as<bool>(false);

      // Parse all fields for this message
      for (const auto & field : mapping["fields"]) {
        PayloadFieldMapping field_mapping;
        auto name = field["name"].as<std::string>();
        auto type = field["type"].as<std::string>("state");
        auto data_type = field["data_type"].as<std::string>("float32");

        // For non-telemetry, prepend module info to the field name (modX__name),
        // this is the module INDEX, not the CAN mod_id
        // For telemetry, also store the topic mapping, 
        // topics are in the format /reseq/modX/name 
        if (type != "telemetry") {
          name = "mod" + std::to_string(i) + "__" + name;
        } else {
          name = "/reseq/mod" + std::to_string(i) + "/" + name;
          topic_mappings.emplace_back(name, data_type);
        }

        field_mapping.name = name;
        field_mapping.mapping_type = parse_type(type);
        field_mapping.mode = field["mode"].as<std::string>("position");
        field_mapping.data_type = data_type;
        field_mapping.offset = field["offset"].as<uint8_t>(0);
        field_mapping.scale = field["scale"].as<double>(1.0);
        field_mapping.bias = field["bias"].as<double>(0.0);

        msg_mapping.fields.push_back(field_mapping);
      }

      // Log the mapping for debugging
      RCLCPP_INFO(
        rclcpp::get_logger("ReseqHardware"), "Mapping CAN 0x%02X%02X", msg_mapping.id.msg_id,
        msg_mapping.id.mod_id);
      can_mappings[msg_mapping.id] = msg_mapping;
    }
  }
  return can_mappings;
}

}  // namespace reseq_hardware
