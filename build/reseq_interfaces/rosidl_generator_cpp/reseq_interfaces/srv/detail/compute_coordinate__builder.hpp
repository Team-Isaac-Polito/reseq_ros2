// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from reseq_interfaces:srv/ComputeCoordinate.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "reseq_interfaces/srv/compute_coordinate.hpp"


#ifndef RESEQ_INTERFACES__SRV__DETAIL__COMPUTE_COORDINATE__BUILDER_HPP_
#define RESEQ_INTERFACES__SRV__DETAIL__COMPUTE_COORDINATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "reseq_interfaces/srv/detail/compute_coordinate__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace reseq_interfaces
{

namespace srv
{

namespace builder
{

class Init_ComputeCoordinate_Request_target_frame
{
public:
  explicit Init_ComputeCoordinate_Request_target_frame(::reseq_interfaces::srv::ComputeCoordinate_Request & msg)
  : msg_(msg)
  {}
  ::reseq_interfaces::srv::ComputeCoordinate_Request target_frame(::reseq_interfaces::srv::ComputeCoordinate_Request::_target_frame_type arg)
  {
    msg_.target_frame = std::move(arg);
    return std::move(msg_);
  }

private:
  ::reseq_interfaces::srv::ComputeCoordinate_Request msg_;
};

class Init_ComputeCoordinate_Request_camera_info
{
public:
  explicit Init_ComputeCoordinate_Request_camera_info(::reseq_interfaces::srv::ComputeCoordinate_Request & msg)
  : msg_(msg)
  {}
  Init_ComputeCoordinate_Request_target_frame camera_info(::reseq_interfaces::srv::ComputeCoordinate_Request::_camera_info_type arg)
  {
    msg_.camera_info = std::move(arg);
    return Init_ComputeCoordinate_Request_target_frame(msg_);
  }

private:
  ::reseq_interfaces::srv::ComputeCoordinate_Request msg_;
};

class Init_ComputeCoordinate_Request_detection
{
public:
  Init_ComputeCoordinate_Request_detection()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ComputeCoordinate_Request_camera_info detection(::reseq_interfaces::srv::ComputeCoordinate_Request::_detection_type arg)
  {
    msg_.detection = std::move(arg);
    return Init_ComputeCoordinate_Request_camera_info(msg_);
  }

private:
  ::reseq_interfaces::srv::ComputeCoordinate_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::reseq_interfaces::srv::ComputeCoordinate_Request>()
{
  return reseq_interfaces::srv::builder::Init_ComputeCoordinate_Request_detection();
}

}  // namespace reseq_interfaces


namespace reseq_interfaces
{

namespace srv
{

namespace builder
{

class Init_ComputeCoordinate_Response_message
{
public:
  explicit Init_ComputeCoordinate_Response_message(::reseq_interfaces::srv::ComputeCoordinate_Response & msg)
  : msg_(msg)
  {}
  ::reseq_interfaces::srv::ComputeCoordinate_Response message(::reseq_interfaces::srv::ComputeCoordinate_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::reseq_interfaces::srv::ComputeCoordinate_Response msg_;
};

class Init_ComputeCoordinate_Response_point
{
public:
  explicit Init_ComputeCoordinate_Response_point(::reseq_interfaces::srv::ComputeCoordinate_Response & msg)
  : msg_(msg)
  {}
  Init_ComputeCoordinate_Response_message point(::reseq_interfaces::srv::ComputeCoordinate_Response::_point_type arg)
  {
    msg_.point = std::move(arg);
    return Init_ComputeCoordinate_Response_message(msg_);
  }

private:
  ::reseq_interfaces::srv::ComputeCoordinate_Response msg_;
};

class Init_ComputeCoordinate_Response_success
{
public:
  Init_ComputeCoordinate_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ComputeCoordinate_Response_point success(::reseq_interfaces::srv::ComputeCoordinate_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_ComputeCoordinate_Response_point(msg_);
  }

private:
  ::reseq_interfaces::srv::ComputeCoordinate_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::reseq_interfaces::srv::ComputeCoordinate_Response>()
{
  return reseq_interfaces::srv::builder::Init_ComputeCoordinate_Response_success();
}

}  // namespace reseq_interfaces


namespace reseq_interfaces
{

namespace srv
{

namespace builder
{

class Init_ComputeCoordinate_Event_response
{
public:
  explicit Init_ComputeCoordinate_Event_response(::reseq_interfaces::srv::ComputeCoordinate_Event & msg)
  : msg_(msg)
  {}
  ::reseq_interfaces::srv::ComputeCoordinate_Event response(::reseq_interfaces::srv::ComputeCoordinate_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::reseq_interfaces::srv::ComputeCoordinate_Event msg_;
};

class Init_ComputeCoordinate_Event_request
{
public:
  explicit Init_ComputeCoordinate_Event_request(::reseq_interfaces::srv::ComputeCoordinate_Event & msg)
  : msg_(msg)
  {}
  Init_ComputeCoordinate_Event_response request(::reseq_interfaces::srv::ComputeCoordinate_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_ComputeCoordinate_Event_response(msg_);
  }

private:
  ::reseq_interfaces::srv::ComputeCoordinate_Event msg_;
};

class Init_ComputeCoordinate_Event_info
{
public:
  Init_ComputeCoordinate_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ComputeCoordinate_Event_request info(::reseq_interfaces::srv::ComputeCoordinate_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_ComputeCoordinate_Event_request(msg_);
  }

private:
  ::reseq_interfaces::srv::ComputeCoordinate_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::reseq_interfaces::srv::ComputeCoordinate_Event>()
{
  return reseq_interfaces::srv::builder::Init_ComputeCoordinate_Event_info();
}

}  // namespace reseq_interfaces

#endif  // RESEQ_INTERFACES__SRV__DETAIL__COMPUTE_COORDINATE__BUILDER_HPP_
