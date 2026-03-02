// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from reseq_interfaces:msg/Remote.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "reseq_interfaces/msg/detail/remote__rosidl_typesupport_introspection_c.h"
#include "reseq_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "reseq_interfaces/msg/detail/remote__functions.h"
#include "reseq_interfaces/msg/detail/remote__struct.h"


// Include directives for member types
// Member `left`
// Member `right`
#include "geometry_msgs/msg/vector3.h"
// Member `left`
// Member `right`
#include "geometry_msgs/msg/detail/vector3__rosidl_typesupport_introspection_c.h"
// Member `buttons`
// Member `switches`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void reseq_interfaces__msg__Remote__rosidl_typesupport_introspection_c__Remote_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  reseq_interfaces__msg__Remote__init(message_memory);
}

void reseq_interfaces__msg__Remote__rosidl_typesupport_introspection_c__Remote_fini_function(void * message_memory)
{
  reseq_interfaces__msg__Remote__fini(message_memory);
}

size_t reseq_interfaces__msg__Remote__rosidl_typesupport_introspection_c__size_function__Remote__buttons(
  const void * untyped_member)
{
  const rosidl_runtime_c__boolean__Sequence * member =
    (const rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return member->size;
}

const void * reseq_interfaces__msg__Remote__rosidl_typesupport_introspection_c__get_const_function__Remote__buttons(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__boolean__Sequence * member =
    (const rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return &member->data[index];
}

void * reseq_interfaces__msg__Remote__rosidl_typesupport_introspection_c__get_function__Remote__buttons(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__boolean__Sequence * member =
    (rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return &member->data[index];
}

void reseq_interfaces__msg__Remote__rosidl_typesupport_introspection_c__fetch_function__Remote__buttons(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const bool * item =
    ((const bool *)
    reseq_interfaces__msg__Remote__rosidl_typesupport_introspection_c__get_const_function__Remote__buttons(untyped_member, index));
  bool * value =
    (bool *)(untyped_value);
  *value = *item;
}

void reseq_interfaces__msg__Remote__rosidl_typesupport_introspection_c__assign_function__Remote__buttons(
  void * untyped_member, size_t index, const void * untyped_value)
{
  bool * item =
    ((bool *)
    reseq_interfaces__msg__Remote__rosidl_typesupport_introspection_c__get_function__Remote__buttons(untyped_member, index));
  const bool * value =
    (const bool *)(untyped_value);
  *item = *value;
}

bool reseq_interfaces__msg__Remote__rosidl_typesupport_introspection_c__resize_function__Remote__buttons(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__boolean__Sequence * member =
    (rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  rosidl_runtime_c__boolean__Sequence__fini(member);
  return rosidl_runtime_c__boolean__Sequence__init(member, size);
}

size_t reseq_interfaces__msg__Remote__rosidl_typesupport_introspection_c__size_function__Remote__switches(
  const void * untyped_member)
{
  const rosidl_runtime_c__boolean__Sequence * member =
    (const rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return member->size;
}

const void * reseq_interfaces__msg__Remote__rosidl_typesupport_introspection_c__get_const_function__Remote__switches(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__boolean__Sequence * member =
    (const rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return &member->data[index];
}

void * reseq_interfaces__msg__Remote__rosidl_typesupport_introspection_c__get_function__Remote__switches(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__boolean__Sequence * member =
    (rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return &member->data[index];
}

void reseq_interfaces__msg__Remote__rosidl_typesupport_introspection_c__fetch_function__Remote__switches(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const bool * item =
    ((const bool *)
    reseq_interfaces__msg__Remote__rosidl_typesupport_introspection_c__get_const_function__Remote__switches(untyped_member, index));
  bool * value =
    (bool *)(untyped_value);
  *value = *item;
}

void reseq_interfaces__msg__Remote__rosidl_typesupport_introspection_c__assign_function__Remote__switches(
  void * untyped_member, size_t index, const void * untyped_value)
{
  bool * item =
    ((bool *)
    reseq_interfaces__msg__Remote__rosidl_typesupport_introspection_c__get_function__Remote__switches(untyped_member, index));
  const bool * value =
    (const bool *)(untyped_value);
  *item = *value;
}

bool reseq_interfaces__msg__Remote__rosidl_typesupport_introspection_c__resize_function__Remote__switches(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__boolean__Sequence * member =
    (rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  rosidl_runtime_c__boolean__Sequence__fini(member);
  return rosidl_runtime_c__boolean__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember reseq_interfaces__msg__Remote__rosidl_typesupport_introspection_c__Remote_message_member_array[4] = {
  {
    "left",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(reseq_interfaces__msg__Remote, left),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "right",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(reseq_interfaces__msg__Remote, right),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "buttons",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(reseq_interfaces__msg__Remote, buttons),  // bytes offset in struct
    NULL,  // default value
    reseq_interfaces__msg__Remote__rosidl_typesupport_introspection_c__size_function__Remote__buttons,  // size() function pointer
    reseq_interfaces__msg__Remote__rosidl_typesupport_introspection_c__get_const_function__Remote__buttons,  // get_const(index) function pointer
    reseq_interfaces__msg__Remote__rosidl_typesupport_introspection_c__get_function__Remote__buttons,  // get(index) function pointer
    reseq_interfaces__msg__Remote__rosidl_typesupport_introspection_c__fetch_function__Remote__buttons,  // fetch(index, &value) function pointer
    reseq_interfaces__msg__Remote__rosidl_typesupport_introspection_c__assign_function__Remote__buttons,  // assign(index, value) function pointer
    reseq_interfaces__msg__Remote__rosidl_typesupport_introspection_c__resize_function__Remote__buttons  // resize(index) function pointer
  },
  {
    "switches",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(reseq_interfaces__msg__Remote, switches),  // bytes offset in struct
    NULL,  // default value
    reseq_interfaces__msg__Remote__rosidl_typesupport_introspection_c__size_function__Remote__switches,  // size() function pointer
    reseq_interfaces__msg__Remote__rosidl_typesupport_introspection_c__get_const_function__Remote__switches,  // get_const(index) function pointer
    reseq_interfaces__msg__Remote__rosidl_typesupport_introspection_c__get_function__Remote__switches,  // get(index) function pointer
    reseq_interfaces__msg__Remote__rosidl_typesupport_introspection_c__fetch_function__Remote__switches,  // fetch(index, &value) function pointer
    reseq_interfaces__msg__Remote__rosidl_typesupport_introspection_c__assign_function__Remote__switches,  // assign(index, value) function pointer
    reseq_interfaces__msg__Remote__rosidl_typesupport_introspection_c__resize_function__Remote__switches  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers reseq_interfaces__msg__Remote__rosidl_typesupport_introspection_c__Remote_message_members = {
  "reseq_interfaces__msg",  // message namespace
  "Remote",  // message name
  4,  // number of fields
  sizeof(reseq_interfaces__msg__Remote),
  false,  // has_any_key_member_
  reseq_interfaces__msg__Remote__rosidl_typesupport_introspection_c__Remote_message_member_array,  // message members
  reseq_interfaces__msg__Remote__rosidl_typesupport_introspection_c__Remote_init_function,  // function to initialize message memory (memory has to be allocated)
  reseq_interfaces__msg__Remote__rosidl_typesupport_introspection_c__Remote_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t reseq_interfaces__msg__Remote__rosidl_typesupport_introspection_c__Remote_message_type_support_handle = {
  0,
  &reseq_interfaces__msg__Remote__rosidl_typesupport_introspection_c__Remote_message_members,
  get_message_typesupport_handle_function,
  &reseq_interfaces__msg__Remote__get_type_hash,
  &reseq_interfaces__msg__Remote__get_type_description,
  &reseq_interfaces__msg__Remote__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_reseq_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, reseq_interfaces, msg, Remote)() {
  reseq_interfaces__msg__Remote__rosidl_typesupport_introspection_c__Remote_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  reseq_interfaces__msg__Remote__rosidl_typesupport_introspection_c__Remote_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  if (!reseq_interfaces__msg__Remote__rosidl_typesupport_introspection_c__Remote_message_type_support_handle.typesupport_identifier) {
    reseq_interfaces__msg__Remote__rosidl_typesupport_introspection_c__Remote_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &reseq_interfaces__msg__Remote__rosidl_typesupport_introspection_c__Remote_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
