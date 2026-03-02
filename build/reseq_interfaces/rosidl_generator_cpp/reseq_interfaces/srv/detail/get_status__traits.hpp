// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from reseq_interfaces:srv/GetStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "reseq_interfaces/srv/get_status.hpp"


#ifndef RESEQ_INTERFACES__SRV__DETAIL__GET_STATUS__TRAITS_HPP_
#define RESEQ_INTERFACES__SRV__DETAIL__GET_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "reseq_interfaces/srv/detail/get_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace reseq_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetStatus_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetStatus_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetStatus_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace reseq_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use reseq_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const reseq_interfaces::srv::GetStatus_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  reseq_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use reseq_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const reseq_interfaces::srv::GetStatus_Request & msg)
{
  return reseq_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<reseq_interfaces::srv::GetStatus_Request>()
{
  return "reseq_interfaces::srv::GetStatus_Request";
}

template<>
inline const char * name<reseq_interfaces::srv::GetStatus_Request>()
{
  return "reseq_interfaces/srv/GetStatus_Request";
}

template<>
struct has_fixed_size<reseq_interfaces::srv::GetStatus_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<reseq_interfaces::srv::GetStatus_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<reseq_interfaces::srv::GetStatus_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace reseq_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetStatus_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: current_mode
  {
    out << "current_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.current_mode, out);
    out << ", ";
  }

  // member: initialized
  {
    out << "initialized: ";
    rosidl_generator_traits::value_to_yaml(msg.initialized, out);
    out << ", ";
  }

  // member: csv_path
  {
    out << "csv_path: ";
    rosidl_generator_traits::value_to_yaml(msg.csv_path, out);
    out << ", ";
  }

  // member: last_error
  {
    out << "last_error: ";
    rosidl_generator_traits::value_to_yaml(msg.last_error, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetStatus_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: current_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.current_mode, out);
    out << "\n";
  }

  // member: initialized
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "initialized: ";
    rosidl_generator_traits::value_to_yaml(msg.initialized, out);
    out << "\n";
  }

  // member: csv_path
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "csv_path: ";
    rosidl_generator_traits::value_to_yaml(msg.csv_path, out);
    out << "\n";
  }

  // member: last_error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "last_error: ";
    rosidl_generator_traits::value_to_yaml(msg.last_error, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetStatus_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace reseq_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use reseq_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const reseq_interfaces::srv::GetStatus_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  reseq_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use reseq_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const reseq_interfaces::srv::GetStatus_Response & msg)
{
  return reseq_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<reseq_interfaces::srv::GetStatus_Response>()
{
  return "reseq_interfaces::srv::GetStatus_Response";
}

template<>
inline const char * name<reseq_interfaces::srv::GetStatus_Response>()
{
  return "reseq_interfaces/srv/GetStatus_Response";
}

template<>
struct has_fixed_size<reseq_interfaces::srv::GetStatus_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<reseq_interfaces::srv::GetStatus_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<reseq_interfaces::srv::GetStatus_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace reseq_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetStatus_Event & msg,
  std::ostream & out)
{
  out << "{";
  // member: info
  {
    out << "info: ";
    to_flow_style_yaml(msg.info, out);
    out << ", ";
  }

  // member: request
  {
    if (msg.request.size() == 0) {
      out << "request: []";
    } else {
      out << "request: [";
      size_t pending_items = msg.request.size();
      for (auto item : msg.request) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: response
  {
    if (msg.response.size() == 0) {
      out << "response: []";
    } else {
      out << "response: [";
      size_t pending_items = msg.response.size();
      for (auto item : msg.response) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetStatus_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "info:\n";
    to_block_style_yaml(msg.info, out, indentation + 2);
  }

  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.request.size() == 0) {
      out << "request: []\n";
    } else {
      out << "request:\n";
      for (auto item : msg.request) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.response.size() == 0) {
      out << "response: []\n";
    } else {
      out << "response:\n";
      for (auto item : msg.response) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetStatus_Event & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace reseq_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use reseq_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const reseq_interfaces::srv::GetStatus_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  reseq_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use reseq_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const reseq_interfaces::srv::GetStatus_Event & msg)
{
  return reseq_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<reseq_interfaces::srv::GetStatus_Event>()
{
  return "reseq_interfaces::srv::GetStatus_Event";
}

template<>
inline const char * name<reseq_interfaces::srv::GetStatus_Event>()
{
  return "reseq_interfaces/srv/GetStatus_Event";
}

template<>
struct has_fixed_size<reseq_interfaces::srv::GetStatus_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<reseq_interfaces::srv::GetStatus_Event>
  : std::integral_constant<bool, has_bounded_size<reseq_interfaces::srv::GetStatus_Request>::value && has_bounded_size<reseq_interfaces::srv::GetStatus_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<reseq_interfaces::srv::GetStatus_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<reseq_interfaces::srv::GetStatus>()
{
  return "reseq_interfaces::srv::GetStatus";
}

template<>
inline const char * name<reseq_interfaces::srv::GetStatus>()
{
  return "reseq_interfaces/srv/GetStatus";
}

template<>
struct has_fixed_size<reseq_interfaces::srv::GetStatus>
  : std::integral_constant<
    bool,
    has_fixed_size<reseq_interfaces::srv::GetStatus_Request>::value &&
    has_fixed_size<reseq_interfaces::srv::GetStatus_Response>::value
  >
{
};

template<>
struct has_bounded_size<reseq_interfaces::srv::GetStatus>
  : std::integral_constant<
    bool,
    has_bounded_size<reseq_interfaces::srv::GetStatus_Request>::value &&
    has_bounded_size<reseq_interfaces::srv::GetStatus_Response>::value
  >
{
};

template<>
struct is_service<reseq_interfaces::srv::GetStatus>
  : std::true_type
{
};

template<>
struct is_service_request<reseq_interfaces::srv::GetStatus_Request>
  : std::true_type
{
};

template<>
struct is_service_response<reseq_interfaces::srv::GetStatus_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // RESEQ_INTERFACES__SRV__DETAIL__GET_STATUS__TRAITS_HPP_
