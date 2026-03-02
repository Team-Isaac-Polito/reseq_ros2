// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from reseq_interfaces:srv/GetStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "reseq_interfaces/srv/get_status.hpp"


#ifndef RESEQ_INTERFACES__SRV__DETAIL__GET_STATUS__BUILDER_HPP_
#define RESEQ_INTERFACES__SRV__DETAIL__GET_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "reseq_interfaces/srv/detail/get_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace reseq_interfaces
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::reseq_interfaces::srv::GetStatus_Request>()
{
  return ::reseq_interfaces::srv::GetStatus_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace reseq_interfaces


namespace reseq_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetStatus_Response_last_error
{
public:
  explicit Init_GetStatus_Response_last_error(::reseq_interfaces::srv::GetStatus_Response & msg)
  : msg_(msg)
  {}
  ::reseq_interfaces::srv::GetStatus_Response last_error(::reseq_interfaces::srv::GetStatus_Response::_last_error_type arg)
  {
    msg_.last_error = std::move(arg);
    return std::move(msg_);
  }

private:
  ::reseq_interfaces::srv::GetStatus_Response msg_;
};

class Init_GetStatus_Response_csv_path
{
public:
  explicit Init_GetStatus_Response_csv_path(::reseq_interfaces::srv::GetStatus_Response & msg)
  : msg_(msg)
  {}
  Init_GetStatus_Response_last_error csv_path(::reseq_interfaces::srv::GetStatus_Response::_csv_path_type arg)
  {
    msg_.csv_path = std::move(arg);
    return Init_GetStatus_Response_last_error(msg_);
  }

private:
  ::reseq_interfaces::srv::GetStatus_Response msg_;
};

class Init_GetStatus_Response_initialized
{
public:
  explicit Init_GetStatus_Response_initialized(::reseq_interfaces::srv::GetStatus_Response & msg)
  : msg_(msg)
  {}
  Init_GetStatus_Response_csv_path initialized(::reseq_interfaces::srv::GetStatus_Response::_initialized_type arg)
  {
    msg_.initialized = std::move(arg);
    return Init_GetStatus_Response_csv_path(msg_);
  }

private:
  ::reseq_interfaces::srv::GetStatus_Response msg_;
};

class Init_GetStatus_Response_current_mode
{
public:
  Init_GetStatus_Response_current_mode()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetStatus_Response_initialized current_mode(::reseq_interfaces::srv::GetStatus_Response::_current_mode_type arg)
  {
    msg_.current_mode = std::move(arg);
    return Init_GetStatus_Response_initialized(msg_);
  }

private:
  ::reseq_interfaces::srv::GetStatus_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::reseq_interfaces::srv::GetStatus_Response>()
{
  return reseq_interfaces::srv::builder::Init_GetStatus_Response_current_mode();
}

}  // namespace reseq_interfaces


namespace reseq_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetStatus_Event_response
{
public:
  explicit Init_GetStatus_Event_response(::reseq_interfaces::srv::GetStatus_Event & msg)
  : msg_(msg)
  {}
  ::reseq_interfaces::srv::GetStatus_Event response(::reseq_interfaces::srv::GetStatus_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::reseq_interfaces::srv::GetStatus_Event msg_;
};

class Init_GetStatus_Event_request
{
public:
  explicit Init_GetStatus_Event_request(::reseq_interfaces::srv::GetStatus_Event & msg)
  : msg_(msg)
  {}
  Init_GetStatus_Event_response request(::reseq_interfaces::srv::GetStatus_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_GetStatus_Event_response(msg_);
  }

private:
  ::reseq_interfaces::srv::GetStatus_Event msg_;
};

class Init_GetStatus_Event_info
{
public:
  Init_GetStatus_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetStatus_Event_request info(::reseq_interfaces::srv::GetStatus_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_GetStatus_Event_request(msg_);
  }

private:
  ::reseq_interfaces::srv::GetStatus_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::reseq_interfaces::srv::GetStatus_Event>()
{
  return reseq_interfaces::srv::builder::Init_GetStatus_Event_info();
}

}  // namespace reseq_interfaces

#endif  // RESEQ_INTERFACES__SRV__DETAIL__GET_STATUS__BUILDER_HPP_
