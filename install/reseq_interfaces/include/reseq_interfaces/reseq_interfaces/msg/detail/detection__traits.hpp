// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from reseq_interfaces:msg/Detection.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "reseq_interfaces/msg/detection.hpp"


#ifndef RESEQ_INTERFACES__MSG__DETAIL__DETECTION__TRAITS_HPP_
#define RESEQ_INTERFACES__MSG__DETAIL__DETECTION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "reseq_interfaces/msg/detail/detection__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace reseq_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Detection & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: detection
  {
    out << "detection: ";
    rosidl_generator_traits::value_to_yaml(msg.detection, out);
    out << ", ";
  }

  // member: type
  {
    out << "type: ";
    rosidl_generator_traits::value_to_yaml(msg.type, out);
    out << ", ";
  }

  // member: name
  {
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << ", ";
  }

  // member: robot
  {
    out << "robot: ";
    rosidl_generator_traits::value_to_yaml(msg.robot, out);
    out << ", ";
  }

  // member: mode
  {
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << ", ";
  }

  // member: confidence
  {
    out << "confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.confidence, out);
    out << ", ";
  }

  // member: xmin
  {
    out << "xmin: ";
    rosidl_generator_traits::value_to_yaml(msg.xmin, out);
    out << ", ";
  }

  // member: ymin
  {
    out << "ymin: ";
    rosidl_generator_traits::value_to_yaml(msg.ymin, out);
    out << ", ";
  }

  // member: width
  {
    out << "width: ";
    rosidl_generator_traits::value_to_yaml(msg.width, out);
    out << ", ";
  }

  // member: height
  {
    out << "height: ";
    rosidl_generator_traits::value_to_yaml(msg.height, out);
    out << ", ";
  }

  // member: depth_center
  {
    out << "depth_center: ";
    rosidl_generator_traits::value_to_yaml(msg.depth_center, out);
    out << ", ";
  }

  // member: camera_frame
  {
    out << "camera_frame: ";
    rosidl_generator_traits::value_to_yaml(msg.camera_frame, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Detection & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: detection
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "detection: ";
    rosidl_generator_traits::value_to_yaml(msg.detection, out);
    out << "\n";
  }

  // member: type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "type: ";
    rosidl_generator_traits::value_to_yaml(msg.type, out);
    out << "\n";
  }

  // member: name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << "\n";
  }

  // member: robot
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot: ";
    rosidl_generator_traits::value_to_yaml(msg.robot, out);
    out << "\n";
  }

  // member: mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << "\n";
  }

  // member: confidence
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.confidence, out);
    out << "\n";
  }

  // member: xmin
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "xmin: ";
    rosidl_generator_traits::value_to_yaml(msg.xmin, out);
    out << "\n";
  }

  // member: ymin
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ymin: ";
    rosidl_generator_traits::value_to_yaml(msg.ymin, out);
    out << "\n";
  }

  // member: width
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "width: ";
    rosidl_generator_traits::value_to_yaml(msg.width, out);
    out << "\n";
  }

  // member: height
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "height: ";
    rosidl_generator_traits::value_to_yaml(msg.height, out);
    out << "\n";
  }

  // member: depth_center
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "depth_center: ";
    rosidl_generator_traits::value_to_yaml(msg.depth_center, out);
    out << "\n";
  }

  // member: camera_frame
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "camera_frame: ";
    rosidl_generator_traits::value_to_yaml(msg.camera_frame, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Detection & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace reseq_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use reseq_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const reseq_interfaces::msg::Detection & msg,
  std::ostream & out, size_t indentation = 0)
{
  reseq_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use reseq_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const reseq_interfaces::msg::Detection & msg)
{
  return reseq_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<reseq_interfaces::msg::Detection>()
{
  return "reseq_interfaces::msg::Detection";
}

template<>
inline const char * name<reseq_interfaces::msg::Detection>()
{
  return "reseq_interfaces/msg/Detection";
}

template<>
struct has_fixed_size<reseq_interfaces::msg::Detection>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<reseq_interfaces::msg::Detection>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<reseq_interfaces::msg::Detection>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RESEQ_INTERFACES__MSG__DETAIL__DETECTION__TRAITS_HPP_
