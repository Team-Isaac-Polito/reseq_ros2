// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from reseq_interfaces:msg/EndEffector.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "reseq_interfaces/msg/detail/end_effector__rosidl_typesupport_introspection_c.h"
#include "reseq_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "reseq_interfaces/msg/detail/end_effector__functions.h"
#include "reseq_interfaces/msg/detail/end_effector__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void reseq_interfaces__msg__EndEffector__rosidl_typesupport_introspection_c__EndEffector_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  reseq_interfaces__msg__EndEffector__init(message_memory);
}

void reseq_interfaces__msg__EndEffector__rosidl_typesupport_introspection_c__EndEffector_fini_function(void * message_memory)
{
  reseq_interfaces__msg__EndEffector__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember reseq_interfaces__msg__EndEffector__rosidl_typesupport_introspection_c__EndEffector_message_member_array[3] = {
  {
    "pitch_vel",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(reseq_interfaces__msg__EndEffector, pitch_vel),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "head_pitch_vel",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(reseq_interfaces__msg__EndEffector, head_pitch_vel),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "head_roll_vel",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(reseq_interfaces__msg__EndEffector, head_roll_vel),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers reseq_interfaces__msg__EndEffector__rosidl_typesupport_introspection_c__EndEffector_message_members = {
  "reseq_interfaces__msg",  // message namespace
  "EndEffector",  // message name
  3,  // number of fields
  sizeof(reseq_interfaces__msg__EndEffector),
  false,  // has_any_key_member_
  reseq_interfaces__msg__EndEffector__rosidl_typesupport_introspection_c__EndEffector_message_member_array,  // message members
  reseq_interfaces__msg__EndEffector__rosidl_typesupport_introspection_c__EndEffector_init_function,  // function to initialize message memory (memory has to be allocated)
  reseq_interfaces__msg__EndEffector__rosidl_typesupport_introspection_c__EndEffector_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t reseq_interfaces__msg__EndEffector__rosidl_typesupport_introspection_c__EndEffector_message_type_support_handle = {
  0,
  &reseq_interfaces__msg__EndEffector__rosidl_typesupport_introspection_c__EndEffector_message_members,
  get_message_typesupport_handle_function,
  &reseq_interfaces__msg__EndEffector__get_type_hash,
  &reseq_interfaces__msg__EndEffector__get_type_description,
  &reseq_interfaces__msg__EndEffector__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_reseq_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, reseq_interfaces, msg, EndEffector)() {
  if (!reseq_interfaces__msg__EndEffector__rosidl_typesupport_introspection_c__EndEffector_message_type_support_handle.typesupport_identifier) {
    reseq_interfaces__msg__EndEffector__rosidl_typesupport_introspection_c__EndEffector_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &reseq_interfaces__msg__EndEffector__rosidl_typesupport_introspection_c__EndEffector_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
