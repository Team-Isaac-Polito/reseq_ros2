#pragma once

#include <stdint.h>                       // for uint8_t
#include <ament_index_cpp/get_package_share_directory.hpp> // for get_package_share_directory
#include <chrono>                         // for milliseconds
#include <cstring>                        // for memcpy, size_t
#include <limits>                         // for numeric_limits
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
#include "reseq_hardware/payload_codec.hpp"  // for FieldDataType
namespace hardware_interface { class CommandInterface; }
namespace hardware_interface { class StateInterface; }
namespace hardware_interface { struct HardwareInfo; }
namespace rclcpp { class Time; }
namespace rclcpp_lifecycle { class State; }

namespace reseq_hardware
{

const uint8_t HANDSHAKE_MSG_ID = 0x22;
const int THROTTLE_WARN = 5000;

/// Format: 6 bytes = 3 × int16_t (little-endian: x, y, z)
const uint8_t IMU_RAW_ACCEL = 0x92;  ///< Accelerometer: 0.061 mg/LSB at ±2g
const uint8_t IMU_RAW_GYRO  = 0x93;  ///< Gyroscope: 0.004375 dps/LSB at 125dps

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
  std::vector<double> command;        ///< Primary command (position for arm, velocity for wheels)
  std::vector<double> command_velocity;  ///< Velocity command (for arm joints with both position+velocity interfaces)
  std::vector<uint8_t> command_position_seeded;  ///< True when position command has been seeded from hardware feedback
};

/**
 * @brief Per-sensor data buffers for IMU state interfaces.
 * Each vector stores values for all sensors packed as [sensor0, sensor1, ...].
 * orientation: 4 values/sensor (x,y,z,w); angular_velocity/linear_acceleration: 3 values/sensor.
 */
struct ImuSensorBuffers
{
  std::vector<double> orientation;           ///< x, y, z, w per sensor
  std::vector<double> angular_velocity;      ///< rad/s per sensor
  std::vector<double> linear_acceleration;   ///< m/s² per sensor
};

/**
 * @brief Information about a registered IMU sensor.
 */
struct ImuSensorInfo
{
  size_t index;       ///< 0-based index into ImuSensorBuffers vectors
  uint8_t module_id;  ///< CAN mod_id (e.g. 0x21 for module 1, mk2)
};

/**
 * @brief Types of mapping for payload fields.
 */
enum class MappingType
{
  JOINT_STATE,
  JOINT_COMMAND,
  GPIO_COMMAND,    ///< Non-joint command (ros2_control <gpio> command interface), e.g. LED, torque
  TOPIC_TELEMETRY
};

/**
 * @brief When a command message is transmitted.
 */
enum class SendMode
{
  CYCLIC,     ///< Sent every command cycle (setpoint streams: wheels, arm, joints).
  ON_CHANGE   ///< Sent only when the encoded payload changes (one-shot actions: torque, LED, beak).
};

/**
 * @brief Describes how a field in a CAN message payload is mapped.
 */
struct PayloadFieldMapping
{
  std::string name;          ///< Joint or topic name.
  MappingType mapping_type;  ///< Type of mapping (joint state, command, or telemetry).
  std::string mode;          ///< Mode string (position, velocity, effort, ...).
  std::string data_type;     ///< Data type as string (float32, uint8, uint16, int32, ...).
  FieldDataType type = FieldDataType::FLOAT32;  ///< Parsed data type.
  uint8_t offset;            ///< Offset in the CAN payload (in B).
  double scale = 1.0;
  double bias = 0.0;
  uint32_t bitmask = 0;      ///< If non-zero, the field is a flag setting/clearing these bits.
  double min = -std::numeric_limits<double>::infinity();  ///< Clamp (raw units) before encoding.
  double max = std::numeric_limits<double>::infinity();   ///< Clamp (raw units) before encoding.
};

/**
 * @brief Mapping of a CAN message to its fields
 */
struct CanMessageMapping
{
  CanID id;                                 ///< CAN identifier.
  uint8_t length = 8;                       ///< Length of the CAN message.
  bool is_command;                          ///< True if this message is a command.
  SendMode send_mode = SendMode::CYCLIC;    ///< Transmission policy for command messages.
  std::vector<PayloadFieldMapping> fields;  ///< Field mappings for the payload.
};

/**
 * @brief Information about a joint, including its index and modes.
 */
struct JointInfo
{
  size_t index;                          ///< Index of the joint.
  std::vector<std::string> cmd_modes;    ///< Command modes for the joint (e.g. position, velocity).
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
   * @brief Returns the command buffer a command field reads from, or nullptr if unknown.
   */
  double * get_command_buffer_ptr(const PayloadFieldMapping & field);

  /**
   * @brief Sends ON_CHANGE command messages whose encoded payload differs from the last one sent.
   *
   * A message is held back while any of its fields has no command yet (NaN), so nothing
   * is sent to the firmware until a controller (or a URDF initial_value) provides a value.
   * On a failed send the message is retried on the next command cycle.
   */
  void write_on_change_messages();

  /**
   * @brief Parses the configuration file for CAN mappings.
   * @param filename Path to the configuration file.
   */
  void parse_config_file(const std::string & filename);

  rclcpp::Clock::SharedPtr clock_ = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);  ///< ROS2 clock.

  int num_modules_ = 0;                 ///< Number of modules.
  int mk_version_ = 2;                  ///< Robot version.
  std::string can_interface_ = "can0";  ///< CAN interface name.
  bool ready_ = false;                  ///< True if hardware is ready.
  std::unique_ptr<CanBus> canbus_;      ///< CAN bus interface.

  std::unordered_map<std::string, JointInfo> joint_info_;  ///< Joint information map.
  std::map<CanID, CanMessageMapping> can_mappings_;        ///< CAN message mappings.
  JointBuffers joint_buffers_;                             ///< Buffers for joint data.
  MessageBuffer recv_buffer_;                              ///< Buffer for received messages.

  std::unordered_map<std::string, ImuSensorInfo> sensor_info_;  ///< IMU sensor information map.
  ImuSensorBuffers sensor_buffers_;                             ///< Buffers for IMU sensor data.

  /// GPIO command interfaces ("gpio_name/interface_name" → index in gpio_commands_).
  std::map<std::string, size_t> gpio_command_index_;
  std::vector<double> gpio_commands_;     ///< GPIO command buffers (NaN = no command yet).

  /// Joints commanded by ON_CHANGE messages: never seeded from feedback, start as NaN.
  std::vector<uint8_t> joint_event_commanded_;

  /// Last payload successfully sent for each ON_CHANGE message.
  std::map<CanID, std::vector<uint8_t>> last_sent_payload_;

  std::chrono::steady_clock::time_point last_write_time_{std::chrono::steady_clock::now()};
  std::chrono::milliseconds command_cycle_{80};    ///< Command cycle duration.
  std::string config_file_ = ament_index_cpp::get_package_share_directory("reseq_hardware") +
    "/config/mappings.yaml";                           ///< Configuration file path.
};
}  // namespace reseq_hardware
