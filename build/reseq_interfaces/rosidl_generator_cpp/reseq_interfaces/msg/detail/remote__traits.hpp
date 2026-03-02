// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from reseq_interfaces:msg/Remote.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "reseq_interfaces/msg/remote.hpp"


#ifndef RESEQ_INTERFACES__MSG__DETAIL__REMOTE__TRAITS_HPP_
#define RESEQ_INTERFACES__MSG__DETAIL__REMOTE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "reseq_interfaces/msg/detail/remote__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'left'
// Member 'right'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace reseq_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Remote & msg,
  std::ostream & out)
{
  out << "{";
  // member: left
  {
    out << "left: ";
    to_flow_style_yaml(msg.left, out);
    out << ", ";
  }

  // member: right
  {
    out << "right: ";
    to_flow_style_yaml(msg.right, out);
    out << ", ";
  }

  // member: buttons
  {
    if (msg.buttons.size() == 0) {
      out << "buttons: []";
    } else {
      out << "buttons: [";
      size_t pending_items = msg.buttons.size();
      for (auto item : msg.buttons) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: switches
  {
    if (msg.switches.size() == 0) {
      out << "switches: []";
    } else {
      out << "switches: [";
      size_t pending_items = msg.switches.size();
      for (auto item : msg.switches) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Remote & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: left
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left:\n";
    to_block_style_yaml(msg.left, out, indentation + 2);
  }

  // member: right
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right:\n";
    to_block_style_yaml(msg.right, out, indentation + 2);
  }

  // member: buttons
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.buttons.size() == 0) {
      out << "buttons: []\n";
    } else {
      out << "buttons:\n";
      for (auto item : msg.buttons) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: switches
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.switches.size() == 0) {
      out << "switches: []\n";
    } else {
      out << "switches:\n";
      for (auto item : msg.switches) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Remote & msg, bool use_flow_style = false)
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
  const reseq_interfaces::msg::Remote & msg,
  std::ostream & out, size_t indentation = 0)
{
  reseq_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use reseq_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const reseq_interfaces::msg::Remote & msg)
{
  return reseq_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<reseq_interfaces::msg::Remote>()
{
  return "reseq_interfaces::msg::Remote";
}

template<>
inline const char * name<reseq_interfaces::msg::Remote>()
{
  return "reseq_interfaces/msg/Remote";
}

template<>
struct has_fixed_size<reseq_interfaces::msg::Remote>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<reseq_interfaces::msg::Remote>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<reseq_interfaces::msg::Remote>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RESEQ_INTERFACES__MSG__DETAIL__REMOTE__TRAITS_HPP_
