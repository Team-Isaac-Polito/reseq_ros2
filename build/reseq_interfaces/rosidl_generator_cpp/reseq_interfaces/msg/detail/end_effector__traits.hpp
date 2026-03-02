// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from reseq_interfaces:msg/EndEffector.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "reseq_interfaces/msg/end_effector.hpp"


#ifndef RESEQ_INTERFACES__MSG__DETAIL__END_EFFECTOR__TRAITS_HPP_
#define RESEQ_INTERFACES__MSG__DETAIL__END_EFFECTOR__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "reseq_interfaces/msg/detail/end_effector__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace reseq_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const EndEffector & msg,
  std::ostream & out)
{
  out << "{";
  // member: pitch_vel
  {
    out << "pitch_vel: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch_vel, out);
    out << ", ";
  }

  // member: head_pitch_vel
  {
    out << "head_pitch_vel: ";
    rosidl_generator_traits::value_to_yaml(msg.head_pitch_vel, out);
    out << ", ";
  }

  // member: head_roll_vel
  {
    out << "head_roll_vel: ";
    rosidl_generator_traits::value_to_yaml(msg.head_roll_vel, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const EndEffector & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: pitch_vel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pitch_vel: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch_vel, out);
    out << "\n";
  }

  // member: head_pitch_vel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "head_pitch_vel: ";
    rosidl_generator_traits::value_to_yaml(msg.head_pitch_vel, out);
    out << "\n";
  }

  // member: head_roll_vel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "head_roll_vel: ";
    rosidl_generator_traits::value_to_yaml(msg.head_roll_vel, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const EndEffector & msg, bool use_flow_style = false)
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
  const reseq_interfaces::msg::EndEffector & msg,
  std::ostream & out, size_t indentation = 0)
{
  reseq_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use reseq_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const reseq_interfaces::msg::EndEffector & msg)
{
  return reseq_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<reseq_interfaces::msg::EndEffector>()
{
  return "reseq_interfaces::msg::EndEffector";
}

template<>
inline const char * name<reseq_interfaces::msg::EndEffector>()
{
  return "reseq_interfaces/msg/EndEffector";
}

template<>
struct has_fixed_size<reseq_interfaces::msg::EndEffector>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<reseq_interfaces::msg::EndEffector>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<reseq_interfaces::msg::EndEffector>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RESEQ_INTERFACES__MSG__DETAIL__END_EFFECTOR__TRAITS_HPP_
