// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from reseq_interfaces:srv/ComputeCoordinate.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "reseq_interfaces/srv/compute_coordinate.hpp"


#ifndef RESEQ_INTERFACES__SRV__DETAIL__COMPUTE_COORDINATE__TRAITS_HPP_
#define RESEQ_INTERFACES__SRV__DETAIL__COMPUTE_COORDINATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "reseq_interfaces/srv/detail/compute_coordinate__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'detection'
#include "reseq_interfaces/msg/detail/detection__traits.hpp"
// Member 'camera_info'
#include "sensor_msgs/msg/detail/camera_info__traits.hpp"

namespace reseq_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const ComputeCoordinate_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: detection
  {
    out << "detection: ";
    to_flow_style_yaml(msg.detection, out);
    out << ", ";
  }

  // member: camera_info
  {
    out << "camera_info: ";
    to_flow_style_yaml(msg.camera_info, out);
    out << ", ";
  }

  // member: target_frame
  {
    out << "target_frame: ";
    rosidl_generator_traits::value_to_yaml(msg.target_frame, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ComputeCoordinate_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: detection
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "detection:\n";
    to_block_style_yaml(msg.detection, out, indentation + 2);
  }

  // member: camera_info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "camera_info:\n";
    to_block_style_yaml(msg.camera_info, out, indentation + 2);
  }

  // member: target_frame
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_frame: ";
    rosidl_generator_traits::value_to_yaml(msg.target_frame, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ComputeCoordinate_Request & msg, bool use_flow_style = false)
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
  const reseq_interfaces::srv::ComputeCoordinate_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  reseq_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use reseq_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const reseq_interfaces::srv::ComputeCoordinate_Request & msg)
{
  return reseq_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<reseq_interfaces::srv::ComputeCoordinate_Request>()
{
  return "reseq_interfaces::srv::ComputeCoordinate_Request";
}

template<>
inline const char * name<reseq_interfaces::srv::ComputeCoordinate_Request>()
{
  return "reseq_interfaces/srv/ComputeCoordinate_Request";
}

template<>
struct has_fixed_size<reseq_interfaces::srv::ComputeCoordinate_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<reseq_interfaces::srv::ComputeCoordinate_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<reseq_interfaces::srv::ComputeCoordinate_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'point'
#include "geometry_msgs/msg/detail/point_stamped__traits.hpp"

namespace reseq_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const ComputeCoordinate_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: point
  {
    out << "point: ";
    to_flow_style_yaml(msg.point, out);
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
  const ComputeCoordinate_Response & msg,
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

  // member: point
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "point:\n";
    to_block_style_yaml(msg.point, out, indentation + 2);
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

inline std::string to_yaml(const ComputeCoordinate_Response & msg, bool use_flow_style = false)
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
  const reseq_interfaces::srv::ComputeCoordinate_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  reseq_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use reseq_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const reseq_interfaces::srv::ComputeCoordinate_Response & msg)
{
  return reseq_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<reseq_interfaces::srv::ComputeCoordinate_Response>()
{
  return "reseq_interfaces::srv::ComputeCoordinate_Response";
}

template<>
inline const char * name<reseq_interfaces::srv::ComputeCoordinate_Response>()
{
  return "reseq_interfaces/srv/ComputeCoordinate_Response";
}

template<>
struct has_fixed_size<reseq_interfaces::srv::ComputeCoordinate_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<reseq_interfaces::srv::ComputeCoordinate_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<reseq_interfaces::srv::ComputeCoordinate_Response>
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
  const ComputeCoordinate_Event & msg,
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
  const ComputeCoordinate_Event & msg,
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

inline std::string to_yaml(const ComputeCoordinate_Event & msg, bool use_flow_style = false)
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
  const reseq_interfaces::srv::ComputeCoordinate_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  reseq_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use reseq_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const reseq_interfaces::srv::ComputeCoordinate_Event & msg)
{
  return reseq_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<reseq_interfaces::srv::ComputeCoordinate_Event>()
{
  return "reseq_interfaces::srv::ComputeCoordinate_Event";
}

template<>
inline const char * name<reseq_interfaces::srv::ComputeCoordinate_Event>()
{
  return "reseq_interfaces/srv/ComputeCoordinate_Event";
}

template<>
struct has_fixed_size<reseq_interfaces::srv::ComputeCoordinate_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<reseq_interfaces::srv::ComputeCoordinate_Event>
  : std::integral_constant<bool, has_bounded_size<reseq_interfaces::srv::ComputeCoordinate_Request>::value && has_bounded_size<reseq_interfaces::srv::ComputeCoordinate_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<reseq_interfaces::srv::ComputeCoordinate_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<reseq_interfaces::srv::ComputeCoordinate>()
{
  return "reseq_interfaces::srv::ComputeCoordinate";
}

template<>
inline const char * name<reseq_interfaces::srv::ComputeCoordinate>()
{
  return "reseq_interfaces/srv/ComputeCoordinate";
}

template<>
struct has_fixed_size<reseq_interfaces::srv::ComputeCoordinate>
  : std::integral_constant<
    bool,
    has_fixed_size<reseq_interfaces::srv::ComputeCoordinate_Request>::value &&
    has_fixed_size<reseq_interfaces::srv::ComputeCoordinate_Response>::value
  >
{
};

template<>
struct has_bounded_size<reseq_interfaces::srv::ComputeCoordinate>
  : std::integral_constant<
    bool,
    has_bounded_size<reseq_interfaces::srv::ComputeCoordinate_Request>::value &&
    has_bounded_size<reseq_interfaces::srv::ComputeCoordinate_Response>::value
  >
{
};

template<>
struct is_service<reseq_interfaces::srv::ComputeCoordinate>
  : std::true_type
{
};

template<>
struct is_service_request<reseq_interfaces::srv::ComputeCoordinate_Request>
  : std::true_type
{
};

template<>
struct is_service_response<reseq_interfaces::srv::ComputeCoordinate_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // RESEQ_INTERFACES__SRV__DETAIL__COMPUTE_COORDINATE__TRAITS_HPP_
