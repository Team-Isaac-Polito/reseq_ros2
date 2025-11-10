#pragma once

#include <stdint.h>                       // for uint8_t
#include <ament_index_cpp/get_package_share_directory.hpp> // for get_package_share_directory
#include <chrono>                         // for milliseconds
#include <cstring>                        // for memcpy, size_t
#include <map>                            // for map
#include <memory>                         // for shared_ptr
#include <rclcpp/duration.hpp>            // for Duration
#include <string>                         // for string
#include <unordered_map>                  // for unordered_map
#include <vector>                         // for vector
#include "hardware_interface/system_interface.hpp" // for SystemInterface
#include "hardware_interface/types/hardware_interface_return_values.hpp" // for return_type
#include "rclcpp/clock.hpp"               // for Clock
#include "rclcpp/macros.hpp"              // for RCLCPP_SHARED_PTR_DEFINITIONS
#include "reseq_hardware/canbus.hpp"      // for CanBus, CanID
#include "reseq_hardware/msg_buffer.hpp"  // for MsgBuffer
namespace hardware_interface { class CommandInterface; }
namespace hardware_interface { class StateInterface; }
namespace hardware_interface { struct HardwareInfo; }
namespace rclcpp { class Time; }
namespace rclcpp_lifecycle { class State; }

namespace reseq_hardware
{

const uint8_t HANDSHAKE_MSG_ID = 0x22;
const int THROTTLE_WARN = 5000;

/**
 * @brief Obtains the module ID from the index and robot version.
 * @param idx Index value.
 * @param mk_version Version of the robot.
 * @return The computed module ID.
 */
constexpr uint8_t idx_to_mod(uint8_t idx, int mk_version)
{
  return (idx & 0x0F) + (mk_version * 0x10);
}

struct JointBuffers
{
  // Kept contiguous for hardware_interface
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> effort;
  std::vector<double> command;
};

/**
 * @brief Types of mapping for payload fields.
 */
enum class MappingType
{
  JOINT_STATE,
  JOINT_COMMAND,
  TOPIC_TELEMETRY
};

/**
 * @brief Describes how a field in a CAN message payload is mapped.
 */
struct PayloadFieldMapping
{
  std::string name;          ///< Joint or topic name.
  MappingType mapping_type;  ///< Type of mapping (joint state, command, or telemetry).
  std::string mode;          ///< Mode string (position, velocity, effort, ...).
  std::string data_type;     ///< Data type as string (float32, ...).
  uint8_t offset;            ///< Offset in the CAN payload (in B).
  double scale = 1.0;
  double bias = 0.0;
};

/**
 * @brief Mapping of a CAN message to its fields
 */
struct CanMessageMapping
{
  CanID id;                                 ///< CAN identifier.
  uint8_t length = 8;                       ///< Length of the CAN message.
  bool is_command;                          ///< True if this message is a command.
  std::vector<PayloadFieldMapping> fields;  ///< Field mappings for the payload.
};

/**
 * @brief Information about a joint, including its index and modes.
 */
struct JointInfo
{
  size_t index;                          ///< Index of the joint.
  std::string cmd_mode;                  ///< Command mode for the joint.
  std::vector<std::string> state_modes;  ///< Supported state modes.
};

/**
 * @brief Hardware interface for the RESE.Q system, implementing ROS2 SystemInterface.
 *
 * This class manages CAN communication, joint state and command buffers, and
 * provides the necessary hooks for ROS2 hardware interface lifecycle.
 */
class ReseqHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ReseqHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state)
  override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state)
  override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state)
  override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state)
  override;
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state)
  override;
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state)
  override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  /**
   * @brief Returns the clock used by the hardware interface.
   */
  rclcpp::Clock::SharedPtr get_clock() const
  {
    return clock_;
  }

private:
  /**
   * @brief Returns a pointer to the buffer for a given state mode and joint index.
   */
  double * get_state_buffer_ptr(const std::string & state_mode, size_t index);

  /**
   * @brief Parses the configuration file for CAN mappings.
   * @param filename Path to the configuration file.
   */
  void parse_config_file(const std::string & filename);

  /**
   * @brief Reads a value of type T from the CAN data buffer.
   * @param data Pointer to the data buffer.
   * @param offset Offset in the buffer to read from.
   * @return The read value as a double.
   * @tparam T The target type to read (e.g., float, int32_t).
   */
  template<typename T>
  inline double read_value(const uint8_t * data, size_t offset)
  {
    T v;
    std::memcpy(&v, data + offset, sizeof(T));
    return static_cast<double>(v);
  }

  /**
   * @brief Writes a value of type T to the CAN data buffer.
   * @param data Pointer to the data buffer.
   * @param offset Offset in the buffer to write to.
   * @param value The value to write, as a double.
   * @tparam T The target type to write (e.g., float, int32_t).
   */
  template<typename T>
  inline void write_value(uint8_t * data, size_t offset, double value)
  {
    T v = static_cast<T>(value);
    std::memcpy(data + offset, &v, sizeof(T));
  }

  rclcpp::Clock::SharedPtr clock_;  ///< ROS2 clock.

  int num_modules_ = 0;                 ///< Number of modules.
  int mk_version_ = 2;                  ///< Robot version.
  std::string can_interface_ = "can0";  ///< CAN interface name.
  bool ready_ = false;                  ///< True if hardware is ready.
  std::unique_ptr<CanBus> canbus_;      ///< CAN bus interface.

  std::unordered_map<std::string, JointInfo> joint_info_;  ///< Joint information map.
  std::map<CanID, CanMessageMapping> can_mappings_;        ///< CAN message mappings.
  JointBuffers joint_buffers_;                             ///< Buffers for joint data.
  MessageBuffer recv_buffer_;                              ///< Buffer for received messages.

  std::chrono::steady_clock::time_point last_write_time_{std::chrono::steady_clock::now()};
  std::chrono::milliseconds command_cycle_{80};    ///< Command cycle duration.
  std::string config_file_ = ament_index_cpp::get_package_share_directory("reseq_hardware") +
    "/config/mappings.yaml";                           ///< Configuration file path.
};
}  // namespace reseq_hardware
