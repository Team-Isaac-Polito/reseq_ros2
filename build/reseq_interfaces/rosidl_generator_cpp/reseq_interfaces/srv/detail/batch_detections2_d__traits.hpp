// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from reseq_interfaces:srv/BatchDetections2D.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "reseq_interfaces/srv/batch_detections2_d.hpp"


#ifndef RESEQ_INTERFACES__SRV__DETAIL__BATCH_DETECTIONS2_D__TRAITS_HPP_
#define RESEQ_INTERFACES__SRV__DETAIL__BATCH_DETECTIONS2_D__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "reseq_interfaces/srv/detail/batch_detections2_d__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'detections'
#include "vision_msgs/msg/detail/detection2_d_array__traits.hpp"

namespace reseq_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const BatchDetections2D_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: robot
  {
    out << "robot: ";
    rosidl_generator_traits::value_to_yaml(msg.robot, out);
    out << ", ";
  }

  // member: mode
  {
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << ", ";
  }

  // member: detections
  {
    out << "detections: ";
    to_flow_style_yaml(msg.detections, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BatchDetections2D_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: robot
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot: ";
    rosidl_generator_traits::value_to_yaml(msg.robot, out);
    out << "\n";
  }

  // member: mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << "\n";
  }

  // member: detections
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "detections:\n";
    to_block_style_yaml(msg.detections, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BatchDetections2D_Request & msg, bool use_flow_style = false)
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
  const reseq_interfaces::srv::BatchDetections2D_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  reseq_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use reseq_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const reseq_interfaces::srv::BatchDetections2D_Request & msg)
{
  return reseq_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<reseq_interfaces::srv::BatchDetections2D_Request>()
{
  return "reseq_interfaces::srv::BatchDetections2D_Request";
}

template<>
inline const char * name<reseq_interfaces::srv::BatchDetections2D_Request>()
{
  return "reseq_interfaces/srv/BatchDetections2D_Request";
}

template<>
struct has_fixed_size<reseq_interfaces::srv::BatchDetections2D_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<reseq_interfaces::srv::BatchDetections2D_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<reseq_interfaces::srv::BatchDetections2D_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace reseq_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const BatchDetections2D_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BatchDetections2D_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BatchDetections2D_Response & msg, bool use_flow_style = false)
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
  const reseq_interfaces::srv::BatchDetections2D_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  reseq_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use reseq_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const reseq_interfaces::srv::BatchDetections2D_Response & msg)
{
  return reseq_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<reseq_interfaces::srv::BatchDetections2D_Response>()
{
  return "reseq_interfaces::srv::BatchDetections2D_Response";
}

template<>
inline const char * name<reseq_interfaces::srv::BatchDetections2D_Response>()
{
  return "reseq_interfaces/srv/BatchDetections2D_Response";
}

template<>
struct has_fixed_size<reseq_interfaces::srv::BatchDetections2D_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<reseq_interfaces::srv::BatchDetections2D_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<reseq_interfaces::srv::BatchDetections2D_Response>
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
  const BatchDetections2D_Event & msg,
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
  const BatchDetections2D_Event & msg,
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

inline std::string to_yaml(const BatchDetections2D_Event & msg, bool use_flow_style = false)
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
  const reseq_interfaces::srv::BatchDetections2D_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  reseq_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use reseq_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const reseq_interfaces::srv::BatchDetections2D_Event & msg)
{
  return reseq_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<reseq_interfaces::srv::BatchDetections2D_Event>()
{
  return "reseq_interfaces::srv::BatchDetections2D_Event";
}

template<>
inline const char * name<reseq_interfaces::srv::BatchDetections2D_Event>()
{
  return "reseq_interfaces/srv/BatchDetections2D_Event";
}

template<>
struct has_fixed_size<reseq_interfaces::srv::BatchDetections2D_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<reseq_interfaces::srv::BatchDetections2D_Event>
  : std::integral_constant<bool, has_bounded_size<reseq_interfaces::srv::BatchDetections2D_Request>::value && has_bounded_size<reseq_interfaces::srv::BatchDetections2D_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<reseq_interfaces::srv::BatchDetections2D_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<reseq_interfaces::srv::BatchDetections2D>()
{
  return "reseq_interfaces::srv::BatchDetections2D";
}

template<>
inline const char * name<reseq_interfaces::srv::BatchDetections2D>()
{
  return "reseq_interfaces/srv/BatchDetections2D";
}

template<>
struct has_fixed_size<reseq_interfaces::srv::BatchDetections2D>
  : std::integral_constant<
    bool,
    has_fixed_size<reseq_interfaces::srv::BatchDetections2D_Request>::value &&
    has_fixed_size<reseq_interfaces::srv::BatchDetections2D_Response>::value
  >
{
};

template<>
struct has_bounded_size<reseq_interfaces::srv::BatchDetections2D>
  : std::integral_constant<
    bool,
    has_bounded_size<reseq_interfaces::srv::BatchDetections2D_Request>::value &&
    has_bounded_size<reseq_interfaces::srv::BatchDetections2D_Response>::value
  >
{
};

template<>
struct is_service<reseq_interfaces::srv::BatchDetections2D>
  : std::true_type
{
};

template<>
struct is_service_request<reseq_interfaces::srv::BatchDetections2D_Request>
  : std::true_type
{
};

template<>
struct is_service_response<reseq_interfaces::srv::BatchDetections2D_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // RESEQ_INTERFACES__SRV__DETAIL__BATCH_DETECTIONS2_D__TRAITS_HPP_
