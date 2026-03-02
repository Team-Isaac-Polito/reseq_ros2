// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from reseq_interfaces:msg/Motors.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "reseq_interfaces/msg/motors.hpp"


#ifndef RESEQ_INTERFACES__MSG__DETAIL__MOTORS__TRAITS_HPP_
#define RESEQ_INTERFACES__MSG__DETAIL__MOTORS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "reseq_interfaces/msg/detail/motors__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace reseq_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Motors & msg,
  std::ostream & out)
{
  out << "{";
  // member: left
  {
    out << "left: ";
    rosidl_generator_traits::value_to_yaml(msg.left, out);
    out << ", ";
  }

  // member: right
  {
    out << "right: ";
    rosidl_generator_traits::value_to_yaml(msg.right, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Motors & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: left
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left: ";
    rosidl_generator_traits::value_to_yaml(msg.left, out);
    out << "\n";
  }

  // member: right
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right: ";
    rosidl_generator_traits::value_to_yaml(msg.right, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Motors & msg, bool use_flow_style = false)
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
  const reseq_interfaces::msg::Motors & msg,
  std::ostream & out, size_t indentation = 0)
{
  reseq_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use reseq_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const reseq_interfaces::msg::Motors & msg)
{
  return reseq_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<reseq_interfaces::msg::Motors>()
{
  return "reseq_interfaces::msg::Motors";
}

template<>
inline const char * name<reseq_interfaces::msg::Motors>()
{
  return "reseq_interfaces/msg/Motors";
}

template<>
struct has_fixed_size<reseq_interfaces::msg::Motors>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<reseq_interfaces::msg::Motors>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<reseq_interfaces::msg::Motors>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RESEQ_INTERFACES__MSG__DETAIL__MOTORS__TRAITS_HPP_
