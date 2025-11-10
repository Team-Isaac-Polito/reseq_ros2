#include "reseq_hardware/reseq.hpp"
#include <stddef.h>                              // for size_t
#include <ratio>                                 // for ratio
#include <rclcpp/clock.hpp>                      // for Clock::SharedPtr
#include <rclcpp/logger.hpp>                     // for get_logger
#include <rclcpp/logging.hpp>                    // for RCLCPP_ERROR, RCLCPP...
#include <type_traits>                           // for add_const<>::type
#include <utility>                               // for pair
#include "hardware_interface/handle.hpp"         // for CommandInterface
#include "hardware_interface/hardware_info.hpp"  // for HardwareInfo, Compon...
#include "pluginlib/class_list_macros.hpp"       // for PLUGINLIB_EXPORT_CLASS
#include "reseq_hardware/config_parser.hpp"      // for parse_can_config_file
namespace rclcpp { class Time; }
namespace rclcpp_lifecycle { class State; }

namespace reseq_hardware
{

// LIFECYCLE NODE INTERFACE METHODS

hardware_interface::CallbackReturn ReseqHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  // (1) Initialize all member variables and process the parameters from the info argument.

  // Verify correctness of info
  if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Extract Parameters (if they exist). num_modules is the only compulsory
  if (info.hardware_parameters.find("num_modules") != info.hardware_parameters.end()) {
    num_modules_ = std::stoi(info.hardware_parameters.at("num_modules"));
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("ReseqHardware"), "num_modules missing!");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info.hardware_parameters.find("can_interface") != info.hardware_parameters.end()) {
    can_interface_ = info.hardware_parameters.at("can_interface");
  }
  if (info.hardware_parameters.find("mk_version") != info.hardware_parameters.end()) {
    mk_version_ = std::stoi(info.hardware_parameters.at("mk_version").substr(2, 1)); // e.g. "mk2" -> 2

  }
  // Initialise the data structures
  size_t num_joints = info.joints.size();

  // TThe double pointers required by ROS2 control are kept contiguous in memory
  joint_buffers_.position.resize(num_joints, 0.0);
  joint_buffers_.velocity.resize(num_joints, 0.0);
  joint_buffers_.effort.resize(num_joints, 0.0);
  joint_buffers_.command.resize(num_joints, 0.0);

  joint_info_.clear();

  // Parse joint information (names, command and state interfaces)
  // Available joints are obtained from the hardware info
  for (size_t i = 0; i < num_joints; i++) {
    const auto & joint = info.joints[i];
    std::string cmd_if = "none";

    if (!joint.command_interfaces.empty()) {
      cmd_if = joint.command_interfaces[0].name; // We only support one command interface

    }
    std::vector<std::string> state_ifs;
    for (const auto & state_if : joint.state_interfaces) {
      state_ifs.push_back(state_if.name);
    }

    joint_info_[joint.name] = JointInfo{i, cmd_if, state_ifs};
  }

  // Parse configuration file for CAN mappings
  parse_config_file(config_file_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ReseqHardware::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  // (2) Set up the communication to the hardware and set everything up so that the hardware can be activated.

  // Set up CAN bus
  canbus_ = std::make_unique<CanBus>(can_interface_);
  if (!canbus_ || !*canbus_) {
    RCLCPP_ERROR(rclcpp::get_logger("ReseqHardware"), "Failed to create CanBus");
    return hardware_interface::CallbackReturn::ERROR;
  }

  canbus_->register_callback(
    [this](auto id, auto data, auto size) {
      recv_buffer_.push_msg(id, data, size);
    });

  // Handshake with all modules (expecting a response to HANDSHAKE_MSG_ID)
  for (int i = 1; i <= num_modules_; i++) {
    uint8_t mod = idx_to_mod(i, mk_version_);

    if (!canbus_->wait_for_message({mod, HANDSHAKE_MSG_ID}, std::chrono::milliseconds(500))) {
      RCLCPP_ERROR(
        rclcpp::get_logger(
          "ReseqHardware"), "Module %d did not respond to handshake", mod);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ReseqHardware::on_cleanup(
  const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  // (-2) Cleanup the hardware interface and prepare for shutdown or re-configuration
  if (canbus_ && *canbus_) {
    canbus_->stop();
  }
  canbus_.reset();
  recv_buffer_.clear();
  ready_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ReseqHardware::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  // (3) Activate the hardware, enabling communication
  if (!canbus_ || !*canbus_) {
    RCLCPP_ERROR(rclcpp::get_logger("ReseqHardware"), "CanBus not initialized");
    return hardware_interface::CallbackReturn::ERROR;
  }

  canbus_->start();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ReseqHardware::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  // (-3) Deactivate the hardware, stopping communication
  canbus_->stop();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ReseqHardware::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  // (-1) Graceful shutdown
  // In our case, just call cleanup
  return on_cleanup(previous_state);
}

hardware_interface::CallbackReturn ReseqHardware::on_error(
  const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  // Error state - try to shutdown cleanly
  RCLCPP_ERROR(
    rclcpp::get_logger(
      "ReseqHardware"), "Error occurred, shutting down hardware interface");
  return on_shutdown(previous_state);
}

// HARDWARE INTERFACE METHODS

hardware_interface::return_type ReseqHardware::read(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  (void)time;
  (void)period;
  // At every cycle started by the controller manager, read the latest data from the hardware
  // In our case we read all messages from the CAN bus buffer and update the joint state buffers accordingly
  auto now = std::chrono::steady_clock::now();

  for (const auto & snap : recv_buffer_.get_all()) {
    // Stale messages are ignored, but we use them to detect communication issues
    if (now - snap.timestamp > std::chrono::milliseconds(500)) {
      RCLCPP_WARN(
        rclcpp::get_logger("ReseqHardware"),
        "Stale CAN message received: %02X%02X", snap.id.mod_id, snap.id.msg_id);
      continue;
    }

    // We identify the mapping for this CAN ID to decode the payload
    const auto & map_it = can_mappings_.find(snap.id);

    if (map_it == can_mappings_.end()) {
      RCLCPP_WARN(
        rclcpp::get_logger("ReseqHardware"),
        "Received unknown CAN message: %02X%02X", snap.id.mod_id, snap.id.msg_id);
      continue;
    }

    const auto & mapping = map_it->second;

    // Decode each field in the mapping
    for (const auto & field : mapping.fields) {
      if (field.mapping_type != MappingType::JOINT_STATE) {
        continue;  // TODO: handle TOPICS (TELEMETRY)

      }
      const auto & jinfo = joint_info_.at(field.name);

      double * buffer_ptr = get_state_buffer_ptr(field.mode, jinfo.index);

      if (!buffer_ptr) {
        continue;
      }

      double value = 0.0;
      if (field.data_type == "float32") {
        value = read_value<float>(snap.data, field.offset);
      } else {
        RCLCPP_WARN(
          rclcpp::get_logger(
            "ReseqHardware"), "Unsupported data type: %s", field.data_type.c_str());
        continue;
      }
      *buffer_ptr = value * field.scale + field.bias;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ReseqHardware::write(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  (void)time;
  (void)period;
  // At every cycle started by the controller manager, write the latest command values to the hardware
  // In our case we iterate along the command mappings and send CAN messages accordingly
  auto now = std::chrono::steady_clock::now();

  // Throttle command sending to the hardware
  if (now - last_write_time_ < command_cycle_) {
    return hardware_interface::return_type::OK;
  }

  // Detect if the control loop is running slower than expected
  if (now - last_write_time_ > command_cycle_ * 1.2) {
    RCLCPP_WARN_SKIPFIRST(
      rclcpp::get_logger("ReseqHardware"),
      "Control loop slowdown detected");
  }

  last_write_time_ = now;

  for (const auto & [can_id, mapping] : can_mappings_) {
    if (!mapping.is_command) {
      continue;
    }

    // Build the message according to the mapping instructions
    uint8_t data[8] = {0};
    for (const auto & field : mapping.fields) {
      if (field.mapping_type != MappingType::JOINT_COMMAND) {
        continue;
      }

      const auto & jinfo = joint_info_.at(field.name);
      double value = joint_buffers_.command[jinfo.index];
      value = (value - field.bias) / field.scale;

      if (field.data_type == "float32") {
        write_value<float>(data, field.offset, value);
      } else {
        RCLCPP_WARN(
          rclcpp::get_logger(
            "ReseqHardware"), "Unsupported data type: %s", field.data_type.c_str());
        continue;
      }
    }

    canbus_->send(can_id, data, mapping.length);
  }
  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> ReseqHardware::export_state_interfaces()
{
  // Create state interfaces for each joint and supported state mode
  std::vector<hardware_interface::StateInterface> state_ifs;
  for (const auto & [joint_name, jinfo] : joint_info_) {
    for (const auto & state_mode : jinfo.state_modes) {
      double * buffer_ptr = get_state_buffer_ptr(state_mode, jinfo.index);
      if (!buffer_ptr) {
        continue;  // Unknown state_mode, skip

      }
      state_ifs.emplace_back(
        hardware_interface::StateInterface(
          joint_name, state_mode,
          buffer_ptr));
    }
  }
  return state_ifs;
}

std::vector<hardware_interface::CommandInterface> ReseqHardware::export_command_interfaces()
{
  // Create command interfaces for each joint with a command mode
  std::vector<hardware_interface::CommandInterface> cmd_ifs;
  for (const auto & [joint_name, jinfo] : joint_info_) {
    if (jinfo.cmd_mode != "none") {
      cmd_ifs.emplace_back(
        hardware_interface::CommandInterface(
          joint_name, jinfo.cmd_mode,
          &joint_buffers_.command[jinfo.index]));
    }
  }
  return cmd_ifs;
}

double * ReseqHardware::get_state_buffer_ptr(const std::string & state_mode, size_t index)
{
  if (state_mode == "position") {
    return &joint_buffers_.position[index];
  } else if (state_mode == "velocity") {
    return &joint_buffers_.velocity[index];
  } else if (state_mode == "effort") {
    return &joint_buffers_.effort[index];
  } else {
    RCLCPP_WARN(rclcpp::get_logger("ReseqHardware"), "Unknown state mode: %s", state_mode.c_str());
  }
  return nullptr;
}

void ReseqHardware::parse_config_file(const std::string & filename)
{
  can_mappings_ = parse_can_config_file(filename, num_modules_, mk_version_);
}

}   // namespace reseq_hardware
PLUGINLIB_EXPORT_CLASS(reseq_hardware::ReseqHardware, hardware_interface::SystemInterface)
