// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from reseq_interfaces:srv/SetMode.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "reseq_interfaces/srv/set_mode.hpp"


#ifndef RESEQ_INTERFACES__SRV__DETAIL__SET_MODE__BUILDER_HPP_
#define RESEQ_INTERFACES__SRV__DETAIL__SET_MODE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "reseq_interfaces/srv/detail/set_mode__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace reseq_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetMode_Request_csv_path
{
public:
  explicit Init_SetMode_Request_csv_path(::reseq_interfaces::srv::SetMode_Request & msg)
  : msg_(msg)
  {}
  ::reseq_interfaces::srv::SetMode_Request csv_path(::reseq_interfaces::srv::SetMode_Request::_csv_path_type arg)
  {
    msg_.csv_path = std::move(arg);
    return std::move(msg_);
  }

private:
  ::reseq_interfaces::srv::SetMode_Request msg_;
};

class Init_SetMode_Request_mode
{
public:
  Init_SetMode_Request_mode()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetMode_Request_csv_path mode(::reseq_interfaces::srv::SetMode_Request::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_SetMode_Request_csv_path(msg_);
  }

private:
  ::reseq_interfaces::srv::SetMode_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::reseq_interfaces::srv::SetMode_Request>()
{
  return reseq_interfaces::srv::builder::Init_SetMode_Request_mode();
}

}  // namespace reseq_interfaces


namespace reseq_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetMode_Response_message
{
public:
  explicit Init_SetMode_Response_message(::reseq_interfaces::srv::SetMode_Response & msg)
  : msg_(msg)
  {}
  ::reseq_interfaces::srv::SetMode_Response message(::reseq_interfaces::srv::SetMode_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::reseq_interfaces::srv::SetMode_Response msg_;
};

class Init_SetMode_Response_previous_mode
{
public:
  explicit Init_SetMode_Response_previous_mode(::reseq_interfaces::srv::SetMode_Response & msg)
  : msg_(msg)
  {}
  Init_SetMode_Response_message previous_mode(::reseq_interfaces::srv::SetMode_Response::_previous_mode_type arg)
  {
    msg_.previous_mode = std::move(arg);
    return Init_SetMode_Response_message(msg_);
  }

private:
  ::reseq_interfaces::srv::SetMode_Response msg_;
};

class Init_SetMode_Response_success
{
public:
  Init_SetMode_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetMode_Response_previous_mode success(::reseq_interfaces::srv::SetMode_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SetMode_Response_previous_mode(msg_);
  }

private:
  ::reseq_interfaces::srv::SetMode_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::reseq_interfaces::srv::SetMode_Response>()
{
  return reseq_interfaces::srv::builder::Init_SetMode_Response_success();
}

}  // namespace reseq_interfaces


namespace reseq_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetMode_Event_response
{
public:
  explicit Init_SetMode_Event_response(::reseq_interfaces::srv::SetMode_Event & msg)
  : msg_(msg)
  {}
  ::reseq_interfaces::srv::SetMode_Event response(::reseq_interfaces::srv::SetMode_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::reseq_interfaces::srv::SetMode_Event msg_;
};

class Init_SetMode_Event_request
{
public:
  explicit Init_SetMode_Event_request(::reseq_interfaces::srv::SetMode_Event & msg)
  : msg_(msg)
  {}
  Init_SetMode_Event_response request(::reseq_interfaces::srv::SetMode_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_SetMode_Event_response(msg_);
  }

private:
  ::reseq_interfaces::srv::SetMode_Event msg_;
};

class Init_SetMode_Event_info
{
public:
  Init_SetMode_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetMode_Event_request info(::reseq_interfaces::srv::SetMode_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_SetMode_Event_request(msg_);
  }

private:
  ::reseq_interfaces::srv::SetMode_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::reseq_interfaces::srv::SetMode_Event>()
{
  return reseq_interfaces::srv::builder::Init_SetMode_Event_info();
}

}  // namespace reseq_interfaces

#endif  // RESEQ_INTERFACES__SRV__DETAIL__SET_MODE__BUILDER_HPP_
