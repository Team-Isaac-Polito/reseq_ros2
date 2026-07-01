#include "reseq_hardware/reseq.hpp"
#include <cmath>                                 // for M_PI
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

  // The double pointers required by ROS2 control are kept contiguous in memory
  joint_buffers_.position.resize(num_joints, 0.0);
  joint_buffers_.velocity.resize(num_joints, 0.0);
  joint_buffers_.effort.resize(num_joints, 0.0);
  joint_buffers_.command.resize(num_joints, 0.0);
  joint_buffers_.command_velocity.resize(num_joints, 0.0);
  joint_buffers_.command_position_seeded.resize(num_joints, 0);

  joint_info_.clear();

  // Parse joint information (names, command and state interfaces)
  // Available joints are obtained from the hardware info
  for (size_t i = 0; i < num_joints; i++) {
    const auto & joint = info.joints[i];

    std::vector<std::string> cmd_ifs;
    for (const auto & cmd_if : joint.command_interfaces) {
      cmd_ifs.push_back(cmd_if.name);
    }

    std::vector<std::string> state_ifs;
    for (const auto & state_if : joint.state_interfaces) {
      state_ifs.push_back(state_if.name);
    }

    joint_info_[joint.name] = JointInfo{i, cmd_ifs, state_ifs};
  }

  // Parse configuration file for CAN mappings
  parse_config_file(config_file_);

  // Parse IMU sensors declared in the ros2_control URDF block
  // Expected sensor names: "imu1", "imu2", etc. (1-based module index)
  const size_t num_sensors = info.sensors.size();
  sensor_buffers_.orientation.resize(num_sensors * 4, 0.0);
  sensor_buffers_.angular_velocity.resize(num_sensors * 3, 0.0);
  sensor_buffers_.linear_acceleration.resize(num_sensors * 3, 0.0);
  // Initialise all orientations to identity quaternion (w = 1, x = y = z = 0)
  for (size_t i = 0; i < num_sensors; i++) {
    sensor_buffers_.orientation[i * 4 + 3] = 1.0;
  }
  sensor_info_.clear();
  for (size_t i = 0; i < num_sensors; i++) {
    const auto & sensor = info.sensors[i];
    // Strip the "imu" prefix to get the 1-based module index
    const uint8_t mod_idx = static_cast<uint8_t>(std::stoul(sensor.name.substr(3)));
    const uint8_t mod_id = idx_to_mod(mod_idx, mk_version_);
    sensor_info_[sensor.name] = ImuSensorInfo{i, mod_id};
    RCLCPP_INFO(
      rclcpp::get_logger("ReseqHardware"),
      "Registered IMU sensor: %s (mod_id=0x%02X)", sensor.name.c_str(), mod_id);
  }

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
    uint8_t can_mod_id = idx_to_mod(i, mk_version_);
    bool responded = false;

    // Retry handshake up to 3 times per module
    for (int attempt = 1; attempt <= 3; attempt++) {
      if (canbus_->wait_for_message({can_mod_id, HANDSHAKE_MSG_ID}, std::chrono::milliseconds(1000))) {
        responded = true;
        break;
      }
      RCLCPP_WARN(
        rclcpp::get_logger("ReseqHardware"),
        "Module %d (CAN ID 0x%02X) did not respond to handshake (attempt %d/3)",
        i, can_mod_id, attempt);
    }

    if (!responded) {
      RCLCPP_ERROR(
        rclcpp::get_logger("ReseqHardware"),
        "Module %d (CAN ID 0x%02X) failed to respond to handshake after 3 attempts",
        i, can_mod_id);
      return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(
      rclcpp::get_logger("ReseqHardware"),
      "Module %d (CAN ID 0x%02X) handshake successful", i, can_mod_id);
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
      const uint8_t mod_idx = (snap.id.mod_id - static_cast<uint8_t>(mk_version_ * 0x10)) & 0x0F;
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("ReseqHardware"),
        *clock_,
        THROTTLE_WARN,
        "Stale CAN message received: %02X%02X (module %d, msg 0x%02X)",
        snap.id.mod_id, snap.id.msg_id,
        mod_idx, snap.id.msg_id);
      continue;
    }

    // Handle raw IMU CAN messages (0x92 accel, 0x93 gyro) from ISM330DLC / LSM6DSL
    // Format: 6 bytes = 3 × int16_t little-endian (x, y, z)
    // Sensitivity: accel 0.061 mg/LSB at ±2g; gyro 0.004375 dps/LSB at 125dps
    if (snap.id.msg_id == IMU_RAW_ACCEL || snap.id.msg_id == IMU_RAW_GYRO) {
      if (snap.size >= 6) {
        const uint8_t mod_idx = (snap.id.mod_id - static_cast<uint8_t>(mk_version_ * 0x10)) & 0x0F;
        const std::string sensor_name = "imu" + std::to_string(mod_idx);
        const auto it = sensor_info_.find(sensor_name);
        if (it != sensor_info_.end()) {
          int16_t rx, ry, rz;
          std::memcpy(&rx, snap.data + 0, sizeof(int16_t));
          std::memcpy(&ry, snap.data + 2, sizeof(int16_t));
          std::memcpy(&rz, snap.data + 4, sizeof(int16_t));
          const size_t idx = it->second.index;
          if (snap.id.msg_id == IMU_RAW_ACCEL) {
            // 0.061 mg/LSB × 1e-3 g/mg × 9.80665 m/s²/g
            constexpr double ACCEL_SCALE = 0.061e-3 * 9.80665;
            sensor_buffers_.linear_acceleration[idx * 3 + 0] = rx * ACCEL_SCALE;
            sensor_buffers_.linear_acceleration[idx * 3 + 1] = ry * ACCEL_SCALE;
            sensor_buffers_.linear_acceleration[idx * 3 + 2] = rz * ACCEL_SCALE;
          } else {
            // 0.004375 dps/LSB × π/180 rad/dps
            const double GYRO_SCALE = 0.004375 * M_PI / 180.0;
            sensor_buffers_.angular_velocity[idx * 3 + 0] = rx * GYRO_SCALE;
            sensor_buffers_.angular_velocity[idx * 3 + 1] = ry * GYRO_SCALE;
            sensor_buffers_.angular_velocity[idx * 3 + 2] = rz * GYRO_SCALE;
          }
        }
      }
      continue;
    }

    // We identify the mapping for this CAN ID to decode the payload
    const auto & map_it = can_mappings_.find(snap.id);

    if (map_it == can_mappings_.end()) {
      const uint8_t mod_idx = (snap.id.mod_id - static_cast<uint8_t>(mk_version_ * 0x10)) & 0x0F;
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("ReseqHardware"),
        *clock_,
        THROTTLE_WARN,
        "Received unknown CAN message: %02X%02X (module %d, msg 0x%02X) — no mapping in config",
        snap.id.mod_id, snap.id.msg_id,
        mod_idx, snap.id.msg_id);
      continue;
    }

    const auto & mapping = map_it->second;

    // Decode each field in the mapping
    for (const auto & field : mapping.fields) {
      if (field.mapping_type != MappingType::JOINT_STATE) {
        continue;  // TODO: handle TOPICS (TELEMETRY)

      }
      const auto jit = joint_info_.find(field.name);
      if (jit == joint_info_.end()) {
        RCLCPP_WARN_THROTTLE(
          rclcpp::get_logger("ReseqHardware"),
          *clock_,
          THROTTLE_WARN,
          "CAN mapping references unknown joint: %s", field.name.c_str());
        continue;
      }
      const auto & jinfo = jit->second;

      double * buffer_ptr = get_state_buffer_ptr(field.mode, jinfo.index);

      if (!buffer_ptr) {
        continue;
      }

      double value = 0.0;
      if (field.data_type == "float32") {
        value = read_value<float>(snap.data, field.offset);
      } else {
        RCLCPP_ERROR(
          rclcpp::get_logger(
            "ReseqHardware"), "Unsupported data type: %s", field.data_type.c_str());
        continue;
      }
      const double scaled_value = value * field.scale + field.bias;
      *buffer_ptr = scaled_value;
      // Seed the position command buffer from actual hardware feedback
      // on first receive, so the arm doesn't jump to 0 on startup.
      if (field.mode == "position" && !joint_buffers_.command_position_seeded[jinfo.index]) {
        joint_buffers_.command[jinfo.index] = scaled_value;
        joint_buffers_.command_position_seeded[jinfo.index] = 1;
      }
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
    bool send_message = true;
    for (const auto & field : mapping.fields) {
      if (field.mapping_type != MappingType::JOINT_COMMAND) {
        continue;
      }

      const auto jit = joint_info_.find(field.name);
      if (jit == joint_info_.end()) {
        RCLCPP_WARN_THROTTLE(
          rclcpp::get_logger("ReseqHardware"),
          *clock_,
          THROTTLE_WARN,
          "CAN mapping references unknown joint: %s", field.name.c_str());
        continue;
      }
      const auto & jinfo = jit->second;

      // Skip sending position commands that haven't been seeded from
      // hardware feedback yet. This prevents the arm from jumping to 0
      // on startup before the first joint state message arrives.
      if (field.mode != "velocity" && !joint_buffers_.command_position_seeded[jinfo.index]) {
        RCLCPP_WARN_THROTTLE(
          rclcpp::get_logger("ReseqHardware"),
          *clock_,
          THROTTLE_WARN,
          "Skipping unseeded position command for joint: %s", field.name.c_str());
        send_message = false;
        break;
      }

      // Read from the correct buffer based on the field's command mode.
      // "velocity" commands come from command_velocity buffer (written by
      // JointGroupVelocityController). All other modes (position, effort)
      // come from the primary command buffer (written by JointTrajectoryController
      // or other position controllers).
      double value = 0.0;
      if (field.mode == "velocity") {
        value = joint_buffers_.command_velocity[jinfo.index];
      } else {
        value = joint_buffers_.command[jinfo.index];
      }
      value = (value - field.bias) / field.scale;

      if (field.data_type == "float32") {
        write_value<float>(data, field.offset, value);
      } else {
        RCLCPP_ERROR(
          rclcpp::get_logger(
            "ReseqHardware"), "Unsupported data type: %s", field.data_type.c_str());
        continue;
      }
    }

    if (send_message) {
      canbus_->send(can_id, data, mapping.length);
    }
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
  // Add IMU sensor state interfaces (10 interfaces per sensor)
  for (const auto & [sensor_name, sinfo] : sensor_info_) {
    const size_t idx = sinfo.index;
    state_ifs.emplace_back(sensor_name, "orientation.x",         &sensor_buffers_.orientation[idx * 4 + 0]);
    state_ifs.emplace_back(sensor_name, "orientation.y",         &sensor_buffers_.orientation[idx * 4 + 1]);
    state_ifs.emplace_back(sensor_name, "orientation.z",         &sensor_buffers_.orientation[idx * 4 + 2]);
    state_ifs.emplace_back(sensor_name, "orientation.w",         &sensor_buffers_.orientation[idx * 4 + 3]);
    state_ifs.emplace_back(sensor_name, "angular_velocity.x",    &sensor_buffers_.angular_velocity[idx * 3 + 0]);
    state_ifs.emplace_back(sensor_name, "angular_velocity.y",    &sensor_buffers_.angular_velocity[idx * 3 + 1]);
    state_ifs.emplace_back(sensor_name, "angular_velocity.z",    &sensor_buffers_.angular_velocity[idx * 3 + 2]);
    state_ifs.emplace_back(sensor_name, "linear_acceleration.x", &sensor_buffers_.linear_acceleration[idx * 3 + 0]);
    state_ifs.emplace_back(sensor_name, "linear_acceleration.y", &sensor_buffers_.linear_acceleration[idx * 3 + 1]);
    state_ifs.emplace_back(sensor_name, "linear_acceleration.z", &sensor_buffers_.linear_acceleration[idx * 3 + 2]);
  }
  return state_ifs;
}

std::vector<hardware_interface::CommandInterface> ReseqHardware::export_command_interfaces()
{
  // Create command interfaces for each joint with command modes
  std::vector<hardware_interface::CommandInterface> cmd_ifs;
  for (const auto & [joint_name, jinfo] : joint_info_) {
    for (const auto & cmd_mode : jinfo.cmd_modes) {
      double * buffer_ptr = nullptr;
      if (cmd_mode == "velocity") {
        // Arm joints expose both position and velocity command interfaces.
        // Velocity commands go to the separate command_velocity buffer.
        buffer_ptr = &joint_buffers_.command_velocity[jinfo.index];
      } else {
        // Position and other commands go to the primary command buffer.
        buffer_ptr = &joint_buffers_.command[jinfo.index];
      }
      cmd_ifs.emplace_back(
        hardware_interface::CommandInterface(
          joint_name, cmd_mode, buffer_ptr));
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
    RCLCPP_ERROR(rclcpp::get_logger("ReseqHardware"), "Unknown state mode: %s", state_mode.c_str());
  }
  return nullptr;
}

void ReseqHardware::parse_config_file(const std::string & filename)
{
  can_mappings_ = parse_can_config_file(filename, num_modules_, mk_version_);
}

}   // namespace reseq_hardware
PLUGINLIB_EXPORT_CLASS(reseq_hardware::ReseqHardware, hardware_interface::SystemInterface)
