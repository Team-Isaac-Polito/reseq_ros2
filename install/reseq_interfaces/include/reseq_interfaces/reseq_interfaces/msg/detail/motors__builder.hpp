// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from reseq_interfaces:msg/Motors.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "reseq_interfaces/msg/motors.hpp"


#ifndef RESEQ_INTERFACES__MSG__DETAIL__MOTORS__BUILDER_HPP_
#define RESEQ_INTERFACES__MSG__DETAIL__MOTORS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "reseq_interfaces/msg/detail/motors__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace reseq_interfaces
{

namespace msg
{

namespace builder
{

class Init_Motors_right
{
public:
  explicit Init_Motors_right(::reseq_interfaces::msg::Motors & msg)
  : msg_(msg)
  {}
  ::reseq_interfaces::msg::Motors right(::reseq_interfaces::msg::Motors::_right_type arg)
  {
    msg_.right = std::move(arg);
    return std::move(msg_);
  }

private:
  ::reseq_interfaces::msg::Motors msg_;
};

class Init_Motors_left
{
public:
  Init_Motors_left()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Motors_right left(::reseq_interfaces::msg::Motors::_left_type arg)
  {
    msg_.left = std::move(arg);
    return Init_Motors_right(msg_);
  }

private:
  ::reseq_interfaces::msg::Motors msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::reseq_interfaces::msg::Motors>()
{
  return reseq_interfaces::msg::builder::Init_Motors_left();
}

}  // namespace reseq_interfaces

#endif  // RESEQ_INTERFACES__MSG__DETAIL__MOTORS__BUILDER_HPP_
