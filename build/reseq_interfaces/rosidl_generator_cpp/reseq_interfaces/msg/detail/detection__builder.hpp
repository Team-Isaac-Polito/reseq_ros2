// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from reseq_interfaces:msg/Detection.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "reseq_interfaces/msg/detection.hpp"


#ifndef RESEQ_INTERFACES__MSG__DETAIL__DETECTION__BUILDER_HPP_
#define RESEQ_INTERFACES__MSG__DETAIL__DETECTION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "reseq_interfaces/msg/detail/detection__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace reseq_interfaces
{

namespace msg
{

namespace builder
{

class Init_Detection_camera_frame
{
public:
  explicit Init_Detection_camera_frame(::reseq_interfaces::msg::Detection & msg)
  : msg_(msg)
  {}
  ::reseq_interfaces::msg::Detection camera_frame(::reseq_interfaces::msg::Detection::_camera_frame_type arg)
  {
    msg_.camera_frame = std::move(arg);
    return std::move(msg_);
  }

private:
  ::reseq_interfaces::msg::Detection msg_;
};

class Init_Detection_depth_center
{
public:
  explicit Init_Detection_depth_center(::reseq_interfaces::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_camera_frame depth_center(::reseq_interfaces::msg::Detection::_depth_center_type arg)
  {
    msg_.depth_center = std::move(arg);
    return Init_Detection_camera_frame(msg_);
  }

private:
  ::reseq_interfaces::msg::Detection msg_;
};

class Init_Detection_height
{
public:
  explicit Init_Detection_height(::reseq_interfaces::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_depth_center height(::reseq_interfaces::msg::Detection::_height_type arg)
  {
    msg_.height = std::move(arg);
    return Init_Detection_depth_center(msg_);
  }

private:
  ::reseq_interfaces::msg::Detection msg_;
};

class Init_Detection_width
{
public:
  explicit Init_Detection_width(::reseq_interfaces::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_height width(::reseq_interfaces::msg::Detection::_width_type arg)
  {
    msg_.width = std::move(arg);
    return Init_Detection_height(msg_);
  }

private:
  ::reseq_interfaces::msg::Detection msg_;
};

class Init_Detection_ymin
{
public:
  explicit Init_Detection_ymin(::reseq_interfaces::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_width ymin(::reseq_interfaces::msg::Detection::_ymin_type arg)
  {
    msg_.ymin = std::move(arg);
    return Init_Detection_width(msg_);
  }

private:
  ::reseq_interfaces::msg::Detection msg_;
};

class Init_Detection_xmin
{
public:
  explicit Init_Detection_xmin(::reseq_interfaces::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_ymin xmin(::reseq_interfaces::msg::Detection::_xmin_type arg)
  {
    msg_.xmin = std::move(arg);
    return Init_Detection_ymin(msg_);
  }

private:
  ::reseq_interfaces::msg::Detection msg_;
};

class Init_Detection_confidence
{
public:
  explicit Init_Detection_confidence(::reseq_interfaces::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_xmin confidence(::reseq_interfaces::msg::Detection::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return Init_Detection_xmin(msg_);
  }

private:
  ::reseq_interfaces::msg::Detection msg_;
};

class Init_Detection_mode
{
public:
  explicit Init_Detection_mode(::reseq_interfaces::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_confidence mode(::reseq_interfaces::msg::Detection::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_Detection_confidence(msg_);
  }

private:
  ::reseq_interfaces::msg::Detection msg_;
};

class Init_Detection_robot
{
public:
  explicit Init_Detection_robot(::reseq_interfaces::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_mode robot(::reseq_interfaces::msg::Detection::_robot_type arg)
  {
    msg_.robot = std::move(arg);
    return Init_Detection_mode(msg_);
  }

private:
  ::reseq_interfaces::msg::Detection msg_;
};

class Init_Detection_name
{
public:
  explicit Init_Detection_name(::reseq_interfaces::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_robot name(::reseq_interfaces::msg::Detection::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_Detection_robot(msg_);
  }

private:
  ::reseq_interfaces::msg::Detection msg_;
};

class Init_Detection_type
{
public:
  explicit Init_Detection_type(::reseq_interfaces::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_name type(::reseq_interfaces::msg::Detection::_type_type arg)
  {
    msg_.type = std::move(arg);
    return Init_Detection_name(msg_);
  }

private:
  ::reseq_interfaces::msg::Detection msg_;
};

class Init_Detection_detection
{
public:
  explicit Init_Detection_detection(::reseq_interfaces::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_type detection(::reseq_interfaces::msg::Detection::_detection_type arg)
  {
    msg_.detection = std::move(arg);
    return Init_Detection_type(msg_);
  }

private:
  ::reseq_interfaces::msg::Detection msg_;
};

class Init_Detection_header
{
public:
  Init_Detection_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Detection_detection header(::reseq_interfaces::msg::Detection::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Detection_detection(msg_);
  }

private:
  ::reseq_interfaces::msg::Detection msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::reseq_interfaces::msg::Detection>()
{
  return reseq_interfaces::msg::builder::Init_Detection_header();
}

}  // namespace reseq_interfaces

#endif  // RESEQ_INTERFACES__MSG__DETAIL__DETECTION__BUILDER_HPP_
