#pragma once

#include <map>     // for map
#include <string>  // for string
namespace reseq_hardware { struct CanID; }
namespace reseq_hardware { struct CanMessageMapping; }

namespace reseq_hardware
{

/**
 * @brief Parses a YAML configuration file and generates CAN message mappings.
 * @param filename Path to the YAML configuration file.
 * @param num_modules Number of modules to generate mappings for.
 * @param mk_version Version of the robot (used for module ID calculation).
 * @return Map of CanID to CanMessageMapping.
 */
std::map<CanID, CanMessageMapping> parse_can_config_file(
  const std::string & filename,
  int num_modules, int mk_version);
}  // namespace reseq_hardware
