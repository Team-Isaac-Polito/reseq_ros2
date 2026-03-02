// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from reseq_interfaces:srv/ComputeCoordinate.idl
// generated code does not contain a copyright notice
#include "reseq_interfaces/srv/detail/compute_coordinate__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "reseq_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "reseq_interfaces/srv/detail/compute_coordinate__struct.h"
#include "reseq_interfaces/srv/detail/compute_coordinate__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "reseq_interfaces/msg/detail/detection__functions.h"  // detection
#include "rosidl_runtime_c/string.h"  // target_frame
#include "rosidl_runtime_c/string_functions.h"  // target_frame
#include "sensor_msgs/msg/detail/camera_info__functions.h"  // camera_info

// forward declare type support functions

bool cdr_serialize_reseq_interfaces__msg__Detection(
  const reseq_interfaces__msg__Detection * ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool cdr_deserialize_reseq_interfaces__msg__Detection(
  eprosima::fastcdr::Cdr & cdr,
  reseq_interfaces__msg__Detection * ros_message);

size_t get_serialized_size_reseq_interfaces__msg__Detection(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_reseq_interfaces__msg__Detection(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool cdr_serialize_key_reseq_interfaces__msg__Detection(
  const reseq_interfaces__msg__Detection * ros_message,
  eprosima::fastcdr::Cdr & cdr);

size_t get_serialized_size_key_reseq_interfaces__msg__Detection(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_key_reseq_interfaces__msg__Detection(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, reseq_interfaces, msg, Detection)();

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_reseq_interfaces
bool cdr_serialize_sensor_msgs__msg__CameraInfo(
  const sensor_msgs__msg__CameraInfo * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_reseq_interfaces
bool cdr_deserialize_sensor_msgs__msg__CameraInfo(
  eprosima::fastcdr::Cdr & cdr,
  sensor_msgs__msg__CameraInfo * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_reseq_interfaces
size_t get_serialized_size_sensor_msgs__msg__CameraInfo(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_reseq_interfaces
size_t max_serialized_size_sensor_msgs__msg__CameraInfo(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_reseq_interfaces
bool cdr_serialize_key_sensor_msgs__msg__CameraInfo(
  const sensor_msgs__msg__CameraInfo * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_reseq_interfaces
size_t get_serialized_size_key_sensor_msgs__msg__CameraInfo(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_reseq_interfaces
size_t max_serialized_size_key_sensor_msgs__msg__CameraInfo(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_reseq_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, sensor_msgs, msg, CameraInfo)();


using _ComputeCoordinate_Request__ros_msg_type = reseq_interfaces__srv__ComputeCoordinate_Request;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_reseq_interfaces
bool cdr_serialize_reseq_interfaces__srv__ComputeCoordinate_Request(
  const reseq_interfaces__srv__ComputeCoordinate_Request * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: detection
  {
    cdr_serialize_reseq_interfaces__msg__Detection(
      &ros_message->detection, cdr);
  }

  // Field name: camera_info
  {
    cdr_serialize_sensor_msgs__msg__CameraInfo(
      &ros_message->camera_info, cdr);
  }

  // Field name: target_frame
  {
    const rosidl_runtime_c__String * str = &ros_message->target_frame;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_reseq_interfaces
bool cdr_deserialize_reseq_interfaces__srv__ComputeCoordinate_Request(
  eprosima::fastcdr::Cdr & cdr,
  reseq_interfaces__srv__ComputeCoordinate_Request * ros_message)
{
  // Field name: detection
  {
    cdr_deserialize_reseq_interfaces__msg__Detection(cdr, &ros_message->detection);
  }

  // Field name: camera_info
  {
    cdr_deserialize_sensor_msgs__msg__CameraInfo(cdr, &ros_message->camera_info);
  }

  // Field name: target_frame
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->target_frame.data) {
      rosidl_runtime_c__String__init(&ros_message->target_frame);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->target_frame,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'target_frame'\n");
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_reseq_interfaces
size_t get_serialized_size_reseq_interfaces__srv__ComputeCoordinate_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ComputeCoordinate_Request__ros_msg_type * ros_message = static_cast<const _ComputeCoordinate_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: detection
  current_alignment += get_serialized_size_reseq_interfaces__msg__Detection(
    &(ros_message->detection), current_alignment);

  // Field name: camera_info
  current_alignment += get_serialized_size_sensor_msgs__msg__CameraInfo(
    &(ros_message->camera_info), current_alignment);

  // Field name: target_frame
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->target_frame.size + 1);

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_reseq_interfaces
size_t max_serialized_size_reseq_interfaces__srv__ComputeCoordinate_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Field name: detection
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_reseq_interfaces__msg__Detection(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: camera_info
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_sensor_msgs__msg__CameraInfo(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: target_frame
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = reseq_interfaces__srv__ComputeCoordinate_Request;
    is_plain =
      (
      offsetof(DataType, target_frame) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_reseq_interfaces
bool cdr_serialize_key_reseq_interfaces__srv__ComputeCoordinate_Request(
  const reseq_interfaces__srv__ComputeCoordinate_Request * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: detection
  {
    cdr_serialize_key_reseq_interfaces__msg__Detection(
      &ros_message->detection, cdr);
  }

  // Field name: camera_info
  {
    cdr_serialize_key_sensor_msgs__msg__CameraInfo(
      &ros_message->camera_info, cdr);
  }

  // Field name: target_frame
  {
    const rosidl_runtime_c__String * str = &ros_message->target_frame;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_reseq_interfaces
size_t get_serialized_size_key_reseq_interfaces__srv__ComputeCoordinate_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ComputeCoordinate_Request__ros_msg_type * ros_message = static_cast<const _ComputeCoordinate_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: detection
  current_alignment += get_serialized_size_key_reseq_interfaces__msg__Detection(
    &(ros_message->detection), current_alignment);

  // Field name: camera_info
  current_alignment += get_serialized_size_key_sensor_msgs__msg__CameraInfo(
    &(ros_message->camera_info), current_alignment);

  // Field name: target_frame
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->target_frame.size + 1);

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_reseq_interfaces
size_t max_serialized_size_key_reseq_interfaces__srv__ComputeCoordinate_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;
  // Field name: detection
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_reseq_interfaces__msg__Detection(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: camera_info
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_sensor_msgs__msg__CameraInfo(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: target_frame
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = reseq_interfaces__srv__ComputeCoordinate_Request;
    is_plain =
      (
      offsetof(DataType, target_frame) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _ComputeCoordinate_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const reseq_interfaces__srv__ComputeCoordinate_Request * ros_message = static_cast<const reseq_interfaces__srv__ComputeCoordinate_Request *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_reseq_interfaces__srv__ComputeCoordinate_Request(ros_message, cdr);
}

static bool _ComputeCoordinate_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  reseq_interfaces__srv__ComputeCoordinate_Request * ros_message = static_cast<reseq_interfaces__srv__ComputeCoordinate_Request *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_reseq_interfaces__srv__ComputeCoordinate_Request(cdr, ros_message);
}

static uint32_t _ComputeCoordinate_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_reseq_interfaces__srv__ComputeCoordinate_Request(
      untyped_ros_message, 0));
}

static size_t _ComputeCoordinate_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_reseq_interfaces__srv__ComputeCoordinate_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_ComputeCoordinate_Request = {
  "reseq_interfaces::srv",
  "ComputeCoordinate_Request",
  _ComputeCoordinate_Request__cdr_serialize,
  _ComputeCoordinate_Request__cdr_deserialize,
  _ComputeCoordinate_Request__get_serialized_size,
  _ComputeCoordinate_Request__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _ComputeCoordinate_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ComputeCoordinate_Request,
  get_message_typesupport_handle_function,
  &reseq_interfaces__srv__ComputeCoordinate_Request__get_type_hash,
  &reseq_interfaces__srv__ComputeCoordinate_Request__get_type_description,
  &reseq_interfaces__srv__ComputeCoordinate_Request__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, reseq_interfaces, srv, ComputeCoordinate_Request)() {
  return &_ComputeCoordinate_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <cstddef>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "reseq_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "reseq_interfaces/srv/detail/compute_coordinate__struct.h"
// already included above
// #include "reseq_interfaces/srv/detail/compute_coordinate__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "geometry_msgs/msg/detail/point_stamped__functions.h"  // point
// already included above
// #include "rosidl_runtime_c/string.h"  // message
// already included above
// #include "rosidl_runtime_c/string_functions.h"  // message

// forward declare type support functions

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_reseq_interfaces
bool cdr_serialize_geometry_msgs__msg__PointStamped(
  const geometry_msgs__msg__PointStamped * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_reseq_interfaces
bool cdr_deserialize_geometry_msgs__msg__PointStamped(
  eprosima::fastcdr::Cdr & cdr,
  geometry_msgs__msg__PointStamped * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_reseq_interfaces
size_t get_serialized_size_geometry_msgs__msg__PointStamped(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_reseq_interfaces
size_t max_serialized_size_geometry_msgs__msg__PointStamped(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_reseq_interfaces
bool cdr_serialize_key_geometry_msgs__msg__PointStamped(
  const geometry_msgs__msg__PointStamped * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_reseq_interfaces
size_t get_serialized_size_key_geometry_msgs__msg__PointStamped(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_reseq_interfaces
size_t max_serialized_size_key_geometry_msgs__msg__PointStamped(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_reseq_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, geometry_msgs, msg, PointStamped)();


using _ComputeCoordinate_Response__ros_msg_type = reseq_interfaces__srv__ComputeCoordinate_Response;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_reseq_interfaces
bool cdr_serialize_reseq_interfaces__srv__ComputeCoordinate_Response(
  const reseq_interfaces__srv__ComputeCoordinate_Response * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: success
  {
    cdr << (ros_message->success ? true : false);
  }

  // Field name: point
  {
    cdr_serialize_geometry_msgs__msg__PointStamped(
      &ros_message->point, cdr);
  }

  // Field name: message
  {
    const rosidl_runtime_c__String * str = &ros_message->message;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_reseq_interfaces
bool cdr_deserialize_reseq_interfaces__srv__ComputeCoordinate_Response(
  eprosima::fastcdr::Cdr & cdr,
  reseq_interfaces__srv__ComputeCoordinate_Response * ros_message)
{
  // Field name: success
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->success = tmp ? true : false;
  }

  // Field name: point
  {
    cdr_deserialize_geometry_msgs__msg__PointStamped(cdr, &ros_message->point);
  }

  // Field name: message
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->message.data) {
      rosidl_runtime_c__String__init(&ros_message->message);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->message,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'message'\n");
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_reseq_interfaces
size_t get_serialized_size_reseq_interfaces__srv__ComputeCoordinate_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ComputeCoordinate_Response__ros_msg_type * ros_message = static_cast<const _ComputeCoordinate_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: success
  {
    size_t item_size = sizeof(ros_message->success);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: point
  current_alignment += get_serialized_size_geometry_msgs__msg__PointStamped(
    &(ros_message->point), current_alignment);

  // Field name: message
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->message.size + 1);

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_reseq_interfaces
size_t max_serialized_size_reseq_interfaces__srv__ComputeCoordinate_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Field name: success
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: point
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_geometry_msgs__msg__PointStamped(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: message
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = reseq_interfaces__srv__ComputeCoordinate_Response;
    is_plain =
      (
      offsetof(DataType, message) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_reseq_interfaces
bool cdr_serialize_key_reseq_interfaces__srv__ComputeCoordinate_Response(
  const reseq_interfaces__srv__ComputeCoordinate_Response * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: success
  {
    cdr << (ros_message->success ? true : false);
  }

  // Field name: point
  {
    cdr_serialize_key_geometry_msgs__msg__PointStamped(
      &ros_message->point, cdr);
  }

  // Field name: message
  {
    const rosidl_runtime_c__String * str = &ros_message->message;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_reseq_interfaces
size_t get_serialized_size_key_reseq_interfaces__srv__ComputeCoordinate_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ComputeCoordinate_Response__ros_msg_type * ros_message = static_cast<const _ComputeCoordinate_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: success
  {
    size_t item_size = sizeof(ros_message->success);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: point
  current_alignment += get_serialized_size_key_geometry_msgs__msg__PointStamped(
    &(ros_message->point), current_alignment);

  // Field name: message
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->message.size + 1);

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_reseq_interfaces
size_t max_serialized_size_key_reseq_interfaces__srv__ComputeCoordinate_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;
  // Field name: success
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: point
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_geometry_msgs__msg__PointStamped(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: message
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = reseq_interfaces__srv__ComputeCoordinate_Response;
    is_plain =
      (
      offsetof(DataType, message) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _ComputeCoordinate_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const reseq_interfaces__srv__ComputeCoordinate_Response * ros_message = static_cast<const reseq_interfaces__srv__ComputeCoordinate_Response *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_reseq_interfaces__srv__ComputeCoordinate_Response(ros_message, cdr);
}

static bool _ComputeCoordinate_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  reseq_interfaces__srv__ComputeCoordinate_Response * ros_message = static_cast<reseq_interfaces__srv__ComputeCoordinate_Response *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_reseq_interfaces__srv__ComputeCoordinate_Response(cdr, ros_message);
}

static uint32_t _ComputeCoordinate_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_reseq_interfaces__srv__ComputeCoordinate_Response(
      untyped_ros_message, 0));
}

static size_t _ComputeCoordinate_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_reseq_interfaces__srv__ComputeCoordinate_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_ComputeCoordinate_Response = {
  "reseq_interfaces::srv",
  "ComputeCoordinate_Response",
  _ComputeCoordinate_Response__cdr_serialize,
  _ComputeCoordinate_Response__cdr_deserialize,
  _ComputeCoordinate_Response__get_serialized_size,
  _ComputeCoordinate_Response__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _ComputeCoordinate_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ComputeCoordinate_Response,
  get_message_typesupport_handle_function,
  &reseq_interfaces__srv__ComputeCoordinate_Response__get_type_hash,
  &reseq_interfaces__srv__ComputeCoordinate_Response__get_type_description,
  &reseq_interfaces__srv__ComputeCoordinate_Response__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, reseq_interfaces, srv, ComputeCoordinate_Response)() {
  return &_ComputeCoordinate_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <cstddef>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "reseq_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "reseq_interfaces/srv/detail/compute_coordinate__struct.h"
// already included above
// #include "reseq_interfaces/srv/detail/compute_coordinate__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "service_msgs/msg/detail/service_event_info__functions.h"  // info

// forward declare type support functions

bool cdr_serialize_reseq_interfaces__srv__ComputeCoordinate_Request(
  const reseq_interfaces__srv__ComputeCoordinate_Request * ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool cdr_deserialize_reseq_interfaces__srv__ComputeCoordinate_Request(
  eprosima::fastcdr::Cdr & cdr,
  reseq_interfaces__srv__ComputeCoordinate_Request * ros_message);

size_t get_serialized_size_reseq_interfaces__srv__ComputeCoordinate_Request(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_reseq_interfaces__srv__ComputeCoordinate_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool cdr_serialize_key_reseq_interfaces__srv__ComputeCoordinate_Request(
  const reseq_interfaces__srv__ComputeCoordinate_Request * ros_message,
  eprosima::fastcdr::Cdr & cdr);

size_t get_serialized_size_key_reseq_interfaces__srv__ComputeCoordinate_Request(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_key_reseq_interfaces__srv__ComputeCoordinate_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, reseq_interfaces, srv, ComputeCoordinate_Request)();

bool cdr_serialize_reseq_interfaces__srv__ComputeCoordinate_Response(
  const reseq_interfaces__srv__ComputeCoordinate_Response * ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool cdr_deserialize_reseq_interfaces__srv__ComputeCoordinate_Response(
  eprosima::fastcdr::Cdr & cdr,
  reseq_interfaces__srv__ComputeCoordinate_Response * ros_message);

size_t get_serialized_size_reseq_interfaces__srv__ComputeCoordinate_Response(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_reseq_interfaces__srv__ComputeCoordinate_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool cdr_serialize_key_reseq_interfaces__srv__ComputeCoordinate_Response(
  const reseq_interfaces__srv__ComputeCoordinate_Response * ros_message,
  eprosima::fastcdr::Cdr & cdr);

size_t get_serialized_size_key_reseq_interfaces__srv__ComputeCoordinate_Response(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_key_reseq_interfaces__srv__ComputeCoordinate_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, reseq_interfaces, srv, ComputeCoordinate_Response)();

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_reseq_interfaces
bool cdr_serialize_service_msgs__msg__ServiceEventInfo(
  const service_msgs__msg__ServiceEventInfo * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_reseq_interfaces
bool cdr_deserialize_service_msgs__msg__ServiceEventInfo(
  eprosima::fastcdr::Cdr & cdr,
  service_msgs__msg__ServiceEventInfo * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_reseq_interfaces
size_t get_serialized_size_service_msgs__msg__ServiceEventInfo(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_reseq_interfaces
size_t max_serialized_size_service_msgs__msg__ServiceEventInfo(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_reseq_interfaces
bool cdr_serialize_key_service_msgs__msg__ServiceEventInfo(
  const service_msgs__msg__ServiceEventInfo * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_reseq_interfaces
size_t get_serialized_size_key_service_msgs__msg__ServiceEventInfo(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_reseq_interfaces
size_t max_serialized_size_key_service_msgs__msg__ServiceEventInfo(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_reseq_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, service_msgs, msg, ServiceEventInfo)();


using _ComputeCoordinate_Event__ros_msg_type = reseq_interfaces__srv__ComputeCoordinate_Event;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_reseq_interfaces
bool cdr_serialize_reseq_interfaces__srv__ComputeCoordinate_Event(
  const reseq_interfaces__srv__ComputeCoordinate_Event * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: info
  {
    cdr_serialize_service_msgs__msg__ServiceEventInfo(
      &ros_message->info, cdr);
  }

  // Field name: request
  {
    size_t size = ros_message->request.size;
    auto array_ptr = ros_message->request.data;
    if (size > 1) {
      fprintf(stderr, "array size exceeds upper bound\n");
      return false;
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_reseq_interfaces__srv__ComputeCoordinate_Request(
        &array_ptr[i], cdr);
    }
  }

  // Field name: response
  {
    size_t size = ros_message->response.size;
    auto array_ptr = ros_message->response.data;
    if (size > 1) {
      fprintf(stderr, "array size exceeds upper bound\n");
      return false;
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_reseq_interfaces__srv__ComputeCoordinate_Response(
        &array_ptr[i], cdr);
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_reseq_interfaces
bool cdr_deserialize_reseq_interfaces__srv__ComputeCoordinate_Event(
  eprosima::fastcdr::Cdr & cdr,
  reseq_interfaces__srv__ComputeCoordinate_Event * ros_message)
{
  // Field name: info
  {
    cdr_deserialize_service_msgs__msg__ServiceEventInfo(cdr, &ros_message->info);
  }

  // Field name: request
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);

    // Check there are at least 'size' remaining bytes in the CDR stream before resizing
    auto old_state = cdr.get_state();
    bool correct_size = cdr.jump(size);
    cdr.set_state(old_state);
    if (!correct_size) {
      fprintf(stderr, "sequence size exceeds remaining buffer\n");
      return false;
    }

    if (ros_message->request.data) {
      reseq_interfaces__srv__ComputeCoordinate_Request__Sequence__fini(&ros_message->request);
    }
    if (!reseq_interfaces__srv__ComputeCoordinate_Request__Sequence__init(&ros_message->request, size)) {
      fprintf(stderr, "failed to create array for field 'request'");
      return false;
    }
    auto array_ptr = ros_message->request.data;
    for (size_t i = 0; i < size; ++i) {
      cdr_deserialize_reseq_interfaces__srv__ComputeCoordinate_Request(cdr, &array_ptr[i]);
    }
  }

  // Field name: response
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);

    // Check there are at least 'size' remaining bytes in the CDR stream before resizing
    auto old_state = cdr.get_state();
    bool correct_size = cdr.jump(size);
    cdr.set_state(old_state);
    if (!correct_size) {
      fprintf(stderr, "sequence size exceeds remaining buffer\n");
      return false;
    }

    if (ros_message->response.data) {
      reseq_interfaces__srv__ComputeCoordinate_Response__Sequence__fini(&ros_message->response);
    }
    if (!reseq_interfaces__srv__ComputeCoordinate_Response__Sequence__init(&ros_message->response, size)) {
      fprintf(stderr, "failed to create array for field 'response'");
      return false;
    }
    auto array_ptr = ros_message->response.data;
    for (size_t i = 0; i < size; ++i) {
      cdr_deserialize_reseq_interfaces__srv__ComputeCoordinate_Response(cdr, &array_ptr[i]);
    }
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_reseq_interfaces
size_t get_serialized_size_reseq_interfaces__srv__ComputeCoordinate_Event(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ComputeCoordinate_Event__ros_msg_type * ros_message = static_cast<const _ComputeCoordinate_Event__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: info
  current_alignment += get_serialized_size_service_msgs__msg__ServiceEventInfo(
    &(ros_message->info), current_alignment);

  // Field name: request
  {
    size_t array_size = ros_message->request.size;
    auto array_ptr = ros_message->request.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_reseq_interfaces__srv__ComputeCoordinate_Request(
        &array_ptr[index], current_alignment);
    }
  }

  // Field name: response
  {
    size_t array_size = ros_message->response.size;
    auto array_ptr = ros_message->response.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_reseq_interfaces__srv__ComputeCoordinate_Response(
        &array_ptr[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_reseq_interfaces
size_t max_serialized_size_reseq_interfaces__srv__ComputeCoordinate_Event(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Field name: info
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_service_msgs__msg__ServiceEventInfo(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: request
  {
    size_t array_size = 1;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_reseq_interfaces__srv__ComputeCoordinate_Request(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: response
  {
    size_t array_size = 1;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_reseq_interfaces__srv__ComputeCoordinate_Response(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = reseq_interfaces__srv__ComputeCoordinate_Event;
    is_plain =
      (
      offsetof(DataType, response) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_reseq_interfaces
bool cdr_serialize_key_reseq_interfaces__srv__ComputeCoordinate_Event(
  const reseq_interfaces__srv__ComputeCoordinate_Event * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: info
  {
    cdr_serialize_key_service_msgs__msg__ServiceEventInfo(
      &ros_message->info, cdr);
  }

  // Field name: request
  {
    size_t size = ros_message->request.size;
    auto array_ptr = ros_message->request.data;
    if (size > 1) {
      fprintf(stderr, "array size exceeds upper bound\n");
      return false;
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_key_reseq_interfaces__srv__ComputeCoordinate_Request(
        &array_ptr[i], cdr);
    }
  }

  // Field name: response
  {
    size_t size = ros_message->response.size;
    auto array_ptr = ros_message->response.data;
    if (size > 1) {
      fprintf(stderr, "array size exceeds upper bound\n");
      return false;
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_key_reseq_interfaces__srv__ComputeCoordinate_Response(
        &array_ptr[i], cdr);
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_reseq_interfaces
size_t get_serialized_size_key_reseq_interfaces__srv__ComputeCoordinate_Event(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ComputeCoordinate_Event__ros_msg_type * ros_message = static_cast<const _ComputeCoordinate_Event__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: info
  current_alignment += get_serialized_size_key_service_msgs__msg__ServiceEventInfo(
    &(ros_message->info), current_alignment);

  // Field name: request
  {
    size_t array_size = ros_message->request.size;
    auto array_ptr = ros_message->request.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_key_reseq_interfaces__srv__ComputeCoordinate_Request(
        &array_ptr[index], current_alignment);
    }
  }

  // Field name: response
  {
    size_t array_size = ros_message->response.size;
    auto array_ptr = ros_message->response.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_key_reseq_interfaces__srv__ComputeCoordinate_Response(
        &array_ptr[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_reseq_interfaces
size_t max_serialized_size_key_reseq_interfaces__srv__ComputeCoordinate_Event(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;
  // Field name: info
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_service_msgs__msg__ServiceEventInfo(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: request
  {
    size_t array_size = 1;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_reseq_interfaces__srv__ComputeCoordinate_Request(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: response
  {
    size_t array_size = 1;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_reseq_interfaces__srv__ComputeCoordinate_Response(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = reseq_interfaces__srv__ComputeCoordinate_Event;
    is_plain =
      (
      offsetof(DataType, response) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _ComputeCoordinate_Event__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const reseq_interfaces__srv__ComputeCoordinate_Event * ros_message = static_cast<const reseq_interfaces__srv__ComputeCoordinate_Event *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_reseq_interfaces__srv__ComputeCoordinate_Event(ros_message, cdr);
}

static bool _ComputeCoordinate_Event__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  reseq_interfaces__srv__ComputeCoordinate_Event * ros_message = static_cast<reseq_interfaces__srv__ComputeCoordinate_Event *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_reseq_interfaces__srv__ComputeCoordinate_Event(cdr, ros_message);
}

static uint32_t _ComputeCoordinate_Event__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_reseq_interfaces__srv__ComputeCoordinate_Event(
      untyped_ros_message, 0));
}

static size_t _ComputeCoordinate_Event__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_reseq_interfaces__srv__ComputeCoordinate_Event(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_ComputeCoordinate_Event = {
  "reseq_interfaces::srv",
  "ComputeCoordinate_Event",
  _ComputeCoordinate_Event__cdr_serialize,
  _ComputeCoordinate_Event__cdr_deserialize,
  _ComputeCoordinate_Event__get_serialized_size,
  _ComputeCoordinate_Event__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _ComputeCoordinate_Event__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ComputeCoordinate_Event,
  get_message_typesupport_handle_function,
  &reseq_interfaces__srv__ComputeCoordinate_Event__get_type_hash,
  &reseq_interfaces__srv__ComputeCoordinate_Event__get_type_description,
  &reseq_interfaces__srv__ComputeCoordinate_Event__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, reseq_interfaces, srv, ComputeCoordinate_Event)() {
  return &_ComputeCoordinate_Event__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "reseq_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "reseq_interfaces/srv/compute_coordinate.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t ComputeCoordinate__callbacks = {
  "reseq_interfaces::srv",
  "ComputeCoordinate",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, reseq_interfaces, srv, ComputeCoordinate_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, reseq_interfaces, srv, ComputeCoordinate_Response)(),
};

static rosidl_service_type_support_t ComputeCoordinate__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &ComputeCoordinate__callbacks,
  get_service_typesupport_handle_function,
  &_ComputeCoordinate_Request__type_support,
  &_ComputeCoordinate_Response__type_support,
  &_ComputeCoordinate_Event__type_support,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    reseq_interfaces,
    srv,
    ComputeCoordinate
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    reseq_interfaces,
    srv,
    ComputeCoordinate
  ),
  &reseq_interfaces__srv__ComputeCoordinate__get_type_hash,
  &reseq_interfaces__srv__ComputeCoordinate__get_type_description,
  &reseq_interfaces__srv__ComputeCoordinate__get_type_description_sources,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, reseq_interfaces, srv, ComputeCoordinate)() {
  return &ComputeCoordinate__handle;
}

#if defined(__cplusplus)
}
#endif
