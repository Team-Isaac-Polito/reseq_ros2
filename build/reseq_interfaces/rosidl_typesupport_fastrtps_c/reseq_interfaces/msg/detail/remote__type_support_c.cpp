// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from reseq_interfaces:msg/Remote.idl
// generated code does not contain a copyright notice
#include "reseq_interfaces/msg/detail/remote__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "reseq_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "reseq_interfaces/msg/detail/remote__struct.h"
#include "reseq_interfaces/msg/detail/remote__functions.h"
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

#include "geometry_msgs/msg/detail/vector3__functions.h"  // left, right
#include "rosidl_runtime_c/primitives_sequence.h"  // buttons, switches
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // buttons, switches

// forward declare type support functions

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_reseq_interfaces
bool cdr_serialize_geometry_msgs__msg__Vector3(
  const geometry_msgs__msg__Vector3 * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_reseq_interfaces
bool cdr_deserialize_geometry_msgs__msg__Vector3(
  eprosima::fastcdr::Cdr & cdr,
  geometry_msgs__msg__Vector3 * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_reseq_interfaces
size_t get_serialized_size_geometry_msgs__msg__Vector3(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_reseq_interfaces
size_t max_serialized_size_geometry_msgs__msg__Vector3(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_reseq_interfaces
bool cdr_serialize_key_geometry_msgs__msg__Vector3(
  const geometry_msgs__msg__Vector3 * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_reseq_interfaces
size_t get_serialized_size_key_geometry_msgs__msg__Vector3(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_reseq_interfaces
size_t max_serialized_size_key_geometry_msgs__msg__Vector3(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_reseq_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Vector3)();


using _Remote__ros_msg_type = reseq_interfaces__msg__Remote;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_reseq_interfaces
bool cdr_serialize_reseq_interfaces__msg__Remote(
  const reseq_interfaces__msg__Remote * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: left
  {
    cdr_serialize_geometry_msgs__msg__Vector3(
      &ros_message->left, cdr);
  }

  // Field name: right
  {
    cdr_serialize_geometry_msgs__msg__Vector3(
      &ros_message->right, cdr);
  }

  // Field name: buttons
  {
    size_t size = ros_message->buttons.size;
    auto array_ptr = ros_message->buttons.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: switches
  {
    size_t size = ros_message->switches.size;
    auto array_ptr = ros_message->switches.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serialize_array(array_ptr, size);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_reseq_interfaces
bool cdr_deserialize_reseq_interfaces__msg__Remote(
  eprosima::fastcdr::Cdr & cdr,
  reseq_interfaces__msg__Remote * ros_message)
{
  // Field name: left
  {
    cdr_deserialize_geometry_msgs__msg__Vector3(cdr, &ros_message->left);
  }

  // Field name: right
  {
    cdr_deserialize_geometry_msgs__msg__Vector3(cdr, &ros_message->right);
  }

  // Field name: buttons
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

    if (ros_message->buttons.data) {
      rosidl_runtime_c__boolean__Sequence__fini(&ros_message->buttons);
    }
    if (!rosidl_runtime_c__boolean__Sequence__init(&ros_message->buttons, size)) {
      fprintf(stderr, "failed to create array for field 'buttons'");
      return false;
    }
    auto array_ptr = ros_message->buttons.data;
    for (size_t i = 0; i < size; ++i) {
      uint8_t tmp;
      cdr >> tmp;
      array_ptr[i] = tmp ? true : false;
    }
  }

  // Field name: switches
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

    if (ros_message->switches.data) {
      rosidl_runtime_c__boolean__Sequence__fini(&ros_message->switches);
    }
    if (!rosidl_runtime_c__boolean__Sequence__init(&ros_message->switches, size)) {
      fprintf(stderr, "failed to create array for field 'switches'");
      return false;
    }
    auto array_ptr = ros_message->switches.data;
    for (size_t i = 0; i < size; ++i) {
      uint8_t tmp;
      cdr >> tmp;
      array_ptr[i] = tmp ? true : false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_reseq_interfaces
size_t get_serialized_size_reseq_interfaces__msg__Remote(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Remote__ros_msg_type * ros_message = static_cast<const _Remote__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: left
  current_alignment += get_serialized_size_geometry_msgs__msg__Vector3(
    &(ros_message->left), current_alignment);

  // Field name: right
  current_alignment += get_serialized_size_geometry_msgs__msg__Vector3(
    &(ros_message->right), current_alignment);

  // Field name: buttons
  {
    size_t array_size = ros_message->buttons.size;
    auto array_ptr = ros_message->buttons.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: switches
  {
    size_t array_size = ros_message->switches.size;
    auto array_ptr = ros_message->switches.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_reseq_interfaces
size_t max_serialized_size_reseq_interfaces__msg__Remote(
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

  // Field name: left
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_geometry_msgs__msg__Vector3(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: right
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_geometry_msgs__msg__Vector3(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: buttons
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: switches
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = reseq_interfaces__msg__Remote;
    is_plain =
      (
      offsetof(DataType, switches) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_reseq_interfaces
bool cdr_serialize_key_reseq_interfaces__msg__Remote(
  const reseq_interfaces__msg__Remote * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: left
  {
    cdr_serialize_key_geometry_msgs__msg__Vector3(
      &ros_message->left, cdr);
  }

  // Field name: right
  {
    cdr_serialize_key_geometry_msgs__msg__Vector3(
      &ros_message->right, cdr);
  }

  // Field name: buttons
  {
    size_t size = ros_message->buttons.size;
    auto array_ptr = ros_message->buttons.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: switches
  {
    size_t size = ros_message->switches.size;
    auto array_ptr = ros_message->switches.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serialize_array(array_ptr, size);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_reseq_interfaces
size_t get_serialized_size_key_reseq_interfaces__msg__Remote(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Remote__ros_msg_type * ros_message = static_cast<const _Remote__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: left
  current_alignment += get_serialized_size_key_geometry_msgs__msg__Vector3(
    &(ros_message->left), current_alignment);

  // Field name: right
  current_alignment += get_serialized_size_key_geometry_msgs__msg__Vector3(
    &(ros_message->right), current_alignment);

  // Field name: buttons
  {
    size_t array_size = ros_message->buttons.size;
    auto array_ptr = ros_message->buttons.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: switches
  {
    size_t array_size = ros_message->switches.size;
    auto array_ptr = ros_message->switches.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_reseq_interfaces
size_t max_serialized_size_key_reseq_interfaces__msg__Remote(
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
  // Field name: left
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_geometry_msgs__msg__Vector3(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: right
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_geometry_msgs__msg__Vector3(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: buttons
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: switches
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = reseq_interfaces__msg__Remote;
    is_plain =
      (
      offsetof(DataType, switches) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _Remote__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const reseq_interfaces__msg__Remote * ros_message = static_cast<const reseq_interfaces__msg__Remote *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_reseq_interfaces__msg__Remote(ros_message, cdr);
}

static bool _Remote__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  reseq_interfaces__msg__Remote * ros_message = static_cast<reseq_interfaces__msg__Remote *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_reseq_interfaces__msg__Remote(cdr, ros_message);
}

static uint32_t _Remote__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_reseq_interfaces__msg__Remote(
      untyped_ros_message, 0));
}

static size_t _Remote__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_reseq_interfaces__msg__Remote(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_Remote = {
  "reseq_interfaces::msg",
  "Remote",
  _Remote__cdr_serialize,
  _Remote__cdr_deserialize,
  _Remote__get_serialized_size,
  _Remote__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _Remote__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Remote,
  get_message_typesupport_handle_function,
  &reseq_interfaces__msg__Remote__get_type_hash,
  &reseq_interfaces__msg__Remote__get_type_description,
  &reseq_interfaces__msg__Remote__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, reseq_interfaces, msg, Remote)() {
  return &_Remote__type_support;
}

#if defined(__cplusplus)
}
#endif
