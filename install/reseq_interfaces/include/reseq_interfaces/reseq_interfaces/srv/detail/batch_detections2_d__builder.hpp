// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from reseq_interfaces:srv/BatchDetections2D.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "reseq_interfaces/srv/batch_detections2_d.hpp"


#ifndef RESEQ_INTERFACES__SRV__DETAIL__BATCH_DETECTIONS2_D__BUILDER_HPP_
#define RESEQ_INTERFACES__SRV__DETAIL__BATCH_DETECTIONS2_D__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "reseq_interfaces/srv/detail/batch_detections2_d__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace reseq_interfaces
{

namespace srv
{

namespace builder
{

class Init_BatchDetections2D_Request_detections
{
public:
  explicit Init_BatchDetections2D_Request_detections(::reseq_interfaces::srv::BatchDetections2D_Request & msg)
  : msg_(msg)
  {}
  ::reseq_interfaces::srv::BatchDetections2D_Request detections(::reseq_interfaces::srv::BatchDetections2D_Request::_detections_type arg)
  {
    msg_.detections = std::move(arg);
    return std::move(msg_);
  }

private:
  ::reseq_interfaces::srv::BatchDetections2D_Request msg_;
};

class Init_BatchDetections2D_Request_mode
{
public:
  explicit Init_BatchDetections2D_Request_mode(::reseq_interfaces::srv::BatchDetections2D_Request & msg)
  : msg_(msg)
  {}
  Init_BatchDetections2D_Request_detections mode(::reseq_interfaces::srv::BatchDetections2D_Request::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_BatchDetections2D_Request_detections(msg_);
  }

private:
  ::reseq_interfaces::srv::BatchDetections2D_Request msg_;
};

class Init_BatchDetections2D_Request_robot
{
public:
  Init_BatchDetections2D_Request_robot()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BatchDetections2D_Request_mode robot(::reseq_interfaces::srv::BatchDetections2D_Request::_robot_type arg)
  {
    msg_.robot = std::move(arg);
    return Init_BatchDetections2D_Request_mode(msg_);
  }

private:
  ::reseq_interfaces::srv::BatchDetections2D_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::reseq_interfaces::srv::BatchDetections2D_Request>()
{
  return reseq_interfaces::srv::builder::Init_BatchDetections2D_Request_robot();
}

}  // namespace reseq_interfaces


namespace reseq_interfaces
{

namespace srv
{

namespace builder
{

class Init_BatchDetections2D_Response_message
{
public:
  explicit Init_BatchDetections2D_Response_message(::reseq_interfaces::srv::BatchDetections2D_Response & msg)
  : msg_(msg)
  {}
  ::reseq_interfaces::srv::BatchDetections2D_Response message(::reseq_interfaces::srv::BatchDetections2D_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::reseq_interfaces::srv::BatchDetections2D_Response msg_;
};

class Init_BatchDetections2D_Response_success
{
public:
  Init_BatchDetections2D_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BatchDetections2D_Response_message success(::reseq_interfaces::srv::BatchDetections2D_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_BatchDetections2D_Response_message(msg_);
  }

private:
  ::reseq_interfaces::srv::BatchDetections2D_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::reseq_interfaces::srv::BatchDetections2D_Response>()
{
  return reseq_interfaces::srv::builder::Init_BatchDetections2D_Response_success();
}

}  // namespace reseq_interfaces


namespace reseq_interfaces
{

namespace srv
{

namespace builder
{

class Init_BatchDetections2D_Event_response
{
public:
  explicit Init_BatchDetections2D_Event_response(::reseq_interfaces::srv::BatchDetections2D_Event & msg)
  : msg_(msg)
  {}
  ::reseq_interfaces::srv::BatchDetections2D_Event response(::reseq_interfaces::srv::BatchDetections2D_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::reseq_interfaces::srv::BatchDetections2D_Event msg_;
};

class Init_BatchDetections2D_Event_request
{
public:
  explicit Init_BatchDetections2D_Event_request(::reseq_interfaces::srv::BatchDetections2D_Event & msg)
  : msg_(msg)
  {}
  Init_BatchDetections2D_Event_response request(::reseq_interfaces::srv::BatchDetections2D_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_BatchDetections2D_Event_response(msg_);
  }

private:
  ::reseq_interfaces::srv::BatchDetections2D_Event msg_;
};

class Init_BatchDetections2D_Event_info
{
public:
  Init_BatchDetections2D_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BatchDetections2D_Event_request info(::reseq_interfaces::srv::BatchDetections2D_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_BatchDetections2D_Event_request(msg_);
  }

private:
  ::reseq_interfaces::srv::BatchDetections2D_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::reseq_interfaces::srv::BatchDetections2D_Event>()
{
  return reseq_interfaces::srv::builder::Init_BatchDetections2D_Event_info();
}

}  // namespace reseq_interfaces

#endif  // RESEQ_INTERFACES__SRV__DETAIL__BATCH_DETECTIONS2_D__BUILDER_HPP_
