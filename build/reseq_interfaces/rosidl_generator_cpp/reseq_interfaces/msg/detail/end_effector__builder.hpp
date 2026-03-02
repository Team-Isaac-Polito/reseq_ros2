// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from reseq_interfaces:msg/EndEffector.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "reseq_interfaces/msg/end_effector.hpp"


#ifndef RESEQ_INTERFACES__MSG__DETAIL__END_EFFECTOR__BUILDER_HPP_
#define RESEQ_INTERFACES__MSG__DETAIL__END_EFFECTOR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "reseq_interfaces/msg/detail/end_effector__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace reseq_interfaces
{

namespace msg
{

namespace builder
{

class Init_EndEffector_head_roll_vel
{
public:
  explicit Init_EndEffector_head_roll_vel(::reseq_interfaces::msg::EndEffector & msg)
  : msg_(msg)
  {}
  ::reseq_interfaces::msg::EndEffector head_roll_vel(::reseq_interfaces::msg::EndEffector::_head_roll_vel_type arg)
  {
    msg_.head_roll_vel = std::move(arg);
    return std::move(msg_);
  }

private:
  ::reseq_interfaces::msg::EndEffector msg_;
};

class Init_EndEffector_head_pitch_vel
{
public:
  explicit Init_EndEffector_head_pitch_vel(::reseq_interfaces::msg::EndEffector & msg)
  : msg_(msg)
  {}
  Init_EndEffector_head_roll_vel head_pitch_vel(::reseq_interfaces::msg::EndEffector::_head_pitch_vel_type arg)
  {
    msg_.head_pitch_vel = std::move(arg);
    return Init_EndEffector_head_roll_vel(msg_);
  }

private:
  ::reseq_interfaces::msg::EndEffector msg_;
};

class Init_EndEffector_pitch_vel
{
public:
  Init_EndEffector_pitch_vel()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_EndEffector_head_pitch_vel pitch_vel(::reseq_interfaces::msg::EndEffector::_pitch_vel_type arg)
  {
    msg_.pitch_vel = std::move(arg);
    return Init_EndEffector_head_pitch_vel(msg_);
  }

private:
  ::reseq_interfaces::msg::EndEffector msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::reseq_interfaces::msg::EndEffector>()
{
  return reseq_interfaces::msg::builder::Init_EndEffector_pitch_vel();
}

}  // namespace reseq_interfaces

#endif  // RESEQ_INTERFACES__MSG__DETAIL__END_EFFECTOR__BUILDER_HPP_
