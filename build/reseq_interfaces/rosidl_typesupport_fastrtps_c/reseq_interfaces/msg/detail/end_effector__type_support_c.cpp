// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from reseq_interfaces:msg/EndEffector.idl
// generated code does not contain a copyright notice
#include "reseq_interfaces/msg/detail/end_effector__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "reseq_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "reseq_interfaces/msg/detail/end_effector__struct.h"
#include "reseq_interfaces/msg/detail/end_effector__functions.h"
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


// forward declare type support functions


using _EndEffector__ros_msg_type = reseq_interfaces__msg__EndEffector;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_reseq_interfaces
bool cdr_serialize_reseq_interfaces__msg__EndEffector(
  const reseq_interfaces__msg__EndEffector * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: pitch_vel
  {
    cdr << ros_message->pitch_vel;
  }

  // Field name: head_pitch_vel
  {
    cdr << ros_message->head_pitch_vel;
  }

  // Field name: head_roll_vel
  {
    cdr << ros_message->head_roll_vel;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_reseq_interfaces
bool cdr_deserialize_reseq_interfaces__msg__EndEffector(
  eprosima::fastcdr::Cdr & cdr,
  reseq_interfaces__msg__EndEffector * ros_message)
{
  // Field name: pitch_vel
  {
    cdr >> ros_message->pitch_vel;
  }

  // Field name: head_pitch_vel
  {
    cdr >> ros_message->head_pitch_vel;
  }

  // Field name: head_roll_vel
  {
    cdr >> ros_message->head_roll_vel;
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_reseq_interfaces
size_t get_serialized_size_reseq_interfaces__msg__EndEffector(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _EndEffector__ros_msg_type * ros_message = static_cast<const _EndEffector__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: pitch_vel
  {
    size_t item_size = sizeof(ros_message->pitch_vel);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: head_pitch_vel
  {
    size_t item_size = sizeof(ros_message->head_pitch_vel);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: head_roll_vel
  {
    size_t item_size = sizeof(ros_message->head_roll_vel);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_reseq_interfaces
size_t max_serialized_size_reseq_interfaces__msg__EndEffector(
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

  // Field name: pitch_vel
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: head_pitch_vel
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: head_roll_vel
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = reseq_interfaces__msg__EndEffector;
    is_plain =
      (
      offsetof(DataType, head_roll_vel) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_reseq_interfaces
bool cdr_serialize_key_reseq_interfaces__msg__EndEffector(
  const reseq_interfaces__msg__EndEffector * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: pitch_vel
  {
    cdr << ros_message->pitch_vel;
  }

  // Field name: head_pitch_vel
  {
    cdr << ros_message->head_pitch_vel;
  }

  // Field name: head_roll_vel
  {
    cdr << ros_message->head_roll_vel;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_reseq_interfaces
size_t get_serialized_size_key_reseq_interfaces__msg__EndEffector(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _EndEffector__ros_msg_type * ros_message = static_cast<const _EndEffector__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: pitch_vel
  {
    size_t item_size = sizeof(ros_message->pitch_vel);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: head_pitch_vel
  {
    size_t item_size = sizeof(ros_message->head_pitch_vel);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: head_roll_vel
  {
    size_t item_size = sizeof(ros_message->head_roll_vel);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_reseq_interfaces
size_t max_serialized_size_key_reseq_interfaces__msg__EndEffector(
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
  // Field name: pitch_vel
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: head_pitch_vel
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: head_roll_vel
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = reseq_interfaces__msg__EndEffector;
    is_plain =
      (
      offsetof(DataType, head_roll_vel) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _EndEffector__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const reseq_interfaces__msg__EndEffector * ros_message = static_cast<const reseq_interfaces__msg__EndEffector *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_reseq_interfaces__msg__EndEffector(ros_message, cdr);
}

static bool _EndEffector__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  reseq_interfaces__msg__EndEffector * ros_message = static_cast<reseq_interfaces__msg__EndEffector *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_reseq_interfaces__msg__EndEffector(cdr, ros_message);
}

static uint32_t _EndEffector__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_reseq_interfaces__msg__EndEffector(
      untyped_ros_message, 0));
}

static size_t _EndEffector__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_reseq_interfaces__msg__EndEffector(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_EndEffector = {
  "reseq_interfaces::msg",
  "EndEffector",
  _EndEffector__cdr_serialize,
  _EndEffector__cdr_deserialize,
  _EndEffector__get_serialized_size,
  _EndEffector__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _EndEffector__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_EndEffector,
  get_message_typesupport_handle_function,
  &reseq_interfaces__msg__EndEffector__get_type_hash,
  &reseq_interfaces__msg__EndEffector__get_type_description,
  &reseq_interfaces__msg__EndEffector__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, reseq_interfaces, msg, EndEffector)() {
  return &_EndEffector__type_support;
}

#if defined(__cplusplus)
}
#endif
