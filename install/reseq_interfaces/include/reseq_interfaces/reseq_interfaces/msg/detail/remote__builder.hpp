// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from reseq_interfaces:msg/Remote.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "reseq_interfaces/msg/remote.hpp"


#ifndef RESEQ_INTERFACES__MSG__DETAIL__REMOTE__BUILDER_HPP_
#define RESEQ_INTERFACES__MSG__DETAIL__REMOTE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "reseq_interfaces/msg/detail/remote__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace reseq_interfaces
{

namespace msg
{

namespace builder
{

class Init_Remote_switches
{
public:
  explicit Init_Remote_switches(::reseq_interfaces::msg::Remote & msg)
  : msg_(msg)
  {}
  ::reseq_interfaces::msg::Remote switches(::reseq_interfaces::msg::Remote::_switches_type arg)
  {
    msg_.switches = std::move(arg);
    return std::move(msg_);
  }

private:
  ::reseq_interfaces::msg::Remote msg_;
};

class Init_Remote_buttons
{
public:
  explicit Init_Remote_buttons(::reseq_interfaces::msg::Remote & msg)
  : msg_(msg)
  {}
  Init_Remote_switches buttons(::reseq_interfaces::msg::Remote::_buttons_type arg)
  {
    msg_.buttons = std::move(arg);
    return Init_Remote_switches(msg_);
  }

private:
  ::reseq_interfaces::msg::Remote msg_;
};

class Init_Remote_right
{
public:
  explicit Init_Remote_right(::reseq_interfaces::msg::Remote & msg)
  : msg_(msg)
  {}
  Init_Remote_buttons right(::reseq_interfaces::msg::Remote::_right_type arg)
  {
    msg_.right = std::move(arg);
    return Init_Remote_buttons(msg_);
  }

private:
  ::reseq_interfaces::msg::Remote msg_;
};

class Init_Remote_left
{
public:
  Init_Remote_left()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Remote_right left(::reseq_interfaces::msg::Remote::_left_type arg)
  {
    msg_.left = std::move(arg);
    return Init_Remote_right(msg_);
  }

private:
  ::reseq_interfaces::msg::Remote msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::reseq_interfaces::msg::Remote>()
{
  return reseq_interfaces::msg::builder::Init_Remote_left();
}

}  // namespace reseq_interfaces

#endif  // RESEQ_INTERFACES__MSG__DETAIL__REMOTE__BUILDER_HPP_
