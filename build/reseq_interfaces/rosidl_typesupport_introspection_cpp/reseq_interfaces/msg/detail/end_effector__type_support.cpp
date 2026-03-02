// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from reseq_interfaces:msg/EndEffector.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "reseq_interfaces/msg/detail/end_effector__functions.h"
#include "reseq_interfaces/msg/detail/end_effector__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace reseq_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void EndEffector_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) reseq_interfaces::msg::EndEffector(_init);
}

void EndEffector_fini_function(void * message_memory)
{
  auto typed_message = static_cast<reseq_interfaces::msg::EndEffector *>(message_memory);
  typed_message->~EndEffector();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember EndEffector_message_member_array[3] = {
  {
    "pitch_vel",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(reseq_interfaces::msg::EndEffector, pitch_vel),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "head_pitch_vel",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(reseq_interfaces::msg::EndEffector, head_pitch_vel),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "head_roll_vel",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(reseq_interfaces::msg::EndEffector, head_roll_vel),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers EndEffector_message_members = {
  "reseq_interfaces::msg",  // message namespace
  "EndEffector",  // message name
  3,  // number of fields
  sizeof(reseq_interfaces::msg::EndEffector),
  false,  // has_any_key_member_
  EndEffector_message_member_array,  // message members
  EndEffector_init_function,  // function to initialize message memory (memory has to be allocated)
  EndEffector_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t EndEffector_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &EndEffector_message_members,
  get_message_typesupport_handle_function,
  &reseq_interfaces__msg__EndEffector__get_type_hash,
  &reseq_interfaces__msg__EndEffector__get_type_description,
  &reseq_interfaces__msg__EndEffector__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace reseq_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<reseq_interfaces::msg::EndEffector>()
{
  return &::reseq_interfaces::msg::rosidl_typesupport_introspection_cpp::EndEffector_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, reseq_interfaces, msg, EndEffector)() {
  return &::reseq_interfaces::msg::rosidl_typesupport_introspection_cpp::EndEffector_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
