// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from reseq_interfaces:srv/ComputeCoordinate.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "reseq_interfaces/srv/detail/compute_coordinate__rosidl_typesupport_introspection_c.h"
#include "reseq_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "reseq_interfaces/srv/detail/compute_coordinate__functions.h"
#include "reseq_interfaces/srv/detail/compute_coordinate__struct.h"


// Include directives for member types
// Member `detection`
#include "reseq_interfaces/msg/detection.h"
// Member `detection`
#include "reseq_interfaces/msg/detail/detection__rosidl_typesupport_introspection_c.h"
// Member `camera_info`
#include "sensor_msgs/msg/camera_info.h"
// Member `camera_info`
#include "sensor_msgs/msg/detail/camera_info__rosidl_typesupport_introspection_c.h"
// Member `target_frame`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void reseq_interfaces__srv__ComputeCoordinate_Request__rosidl_typesupport_introspection_c__ComputeCoordinate_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  reseq_interfaces__srv__ComputeCoordinate_Request__init(message_memory);
}

void reseq_interfaces__srv__ComputeCoordinate_Request__rosidl_typesupport_introspection_c__ComputeCoordinate_Request_fini_function(void * message_memory)
{
  reseq_interfaces__srv__ComputeCoordinate_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember reseq_interfaces__srv__ComputeCoordinate_Request__rosidl_typesupport_introspection_c__ComputeCoordinate_Request_message_member_array[3] = {
  {
    "detection",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(reseq_interfaces__srv__ComputeCoordinate_Request, detection),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "camera_info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(reseq_interfaces__srv__ComputeCoordinate_Request, camera_info),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "target_frame",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(reseq_interfaces__srv__ComputeCoordinate_Request, target_frame),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers reseq_interfaces__srv__ComputeCoordinate_Request__rosidl_typesupport_introspection_c__ComputeCoordinate_Request_message_members = {
  "reseq_interfaces__srv",  // message namespace
  "ComputeCoordinate_Request",  // message name
  3,  // number of fields
  sizeof(reseq_interfaces__srv__ComputeCoordinate_Request),
  false,  // has_any_key_member_
  reseq_interfaces__srv__ComputeCoordinate_Request__rosidl_typesupport_introspection_c__ComputeCoordinate_Request_message_member_array,  // message members
  reseq_interfaces__srv__ComputeCoordinate_Request__rosidl_typesupport_introspection_c__ComputeCoordinate_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  reseq_interfaces__srv__ComputeCoordinate_Request__rosidl_typesupport_introspection_c__ComputeCoordinate_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t reseq_interfaces__srv__ComputeCoordinate_Request__rosidl_typesupport_introspection_c__ComputeCoordinate_Request_message_type_support_handle = {
  0,
  &reseq_interfaces__srv__ComputeCoordinate_Request__rosidl_typesupport_introspection_c__ComputeCoordinate_Request_message_members,
  get_message_typesupport_handle_function,
  &reseq_interfaces__srv__ComputeCoordinate_Request__get_type_hash,
  &reseq_interfaces__srv__ComputeCoordinate_Request__get_type_description,
  &reseq_interfaces__srv__ComputeCoordinate_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_reseq_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, reseq_interfaces, srv, ComputeCoordinate_Request)() {
  reseq_interfaces__srv__ComputeCoordinate_Request__rosidl_typesupport_introspection_c__ComputeCoordinate_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, reseq_interfaces, msg, Detection)();
  reseq_interfaces__srv__ComputeCoordinate_Request__rosidl_typesupport_introspection_c__ComputeCoordinate_Request_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, CameraInfo)();
  if (!reseq_interfaces__srv__ComputeCoordinate_Request__rosidl_typesupport_introspection_c__ComputeCoordinate_Request_message_type_support_handle.typesupport_identifier) {
    reseq_interfaces__srv__ComputeCoordinate_Request__rosidl_typesupport_introspection_c__ComputeCoordinate_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &reseq_interfaces__srv__ComputeCoordinate_Request__rosidl_typesupport_introspection_c__ComputeCoordinate_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "reseq_interfaces/srv/detail/compute_coordinate__rosidl_typesupport_introspection_c.h"
// already included above
// #include "reseq_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "reseq_interfaces/srv/detail/compute_coordinate__functions.h"
// already included above
// #include "reseq_interfaces/srv/detail/compute_coordinate__struct.h"


// Include directives for member types
// Member `point`
#include "geometry_msgs/msg/point_stamped.h"
// Member `point`
#include "geometry_msgs/msg/detail/point_stamped__rosidl_typesupport_introspection_c.h"
// Member `message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void reseq_interfaces__srv__ComputeCoordinate_Response__rosidl_typesupport_introspection_c__ComputeCoordinate_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  reseq_interfaces__srv__ComputeCoordinate_Response__init(message_memory);
}

void reseq_interfaces__srv__ComputeCoordinate_Response__rosidl_typesupport_introspection_c__ComputeCoordinate_Response_fini_function(void * message_memory)
{
  reseq_interfaces__srv__ComputeCoordinate_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember reseq_interfaces__srv__ComputeCoordinate_Response__rosidl_typesupport_introspection_c__ComputeCoordinate_Response_message_member_array[3] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(reseq_interfaces__srv__ComputeCoordinate_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "point",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(reseq_interfaces__srv__ComputeCoordinate_Response, point),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(reseq_interfaces__srv__ComputeCoordinate_Response, message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers reseq_interfaces__srv__ComputeCoordinate_Response__rosidl_typesupport_introspection_c__ComputeCoordinate_Response_message_members = {
  "reseq_interfaces__srv",  // message namespace
  "ComputeCoordinate_Response",  // message name
  3,  // number of fields
  sizeof(reseq_interfaces__srv__ComputeCoordinate_Response),
  false,  // has_any_key_member_
  reseq_interfaces__srv__ComputeCoordinate_Response__rosidl_typesupport_introspection_c__ComputeCoordinate_Response_message_member_array,  // message members
  reseq_interfaces__srv__ComputeCoordinate_Response__rosidl_typesupport_introspection_c__ComputeCoordinate_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  reseq_interfaces__srv__ComputeCoordinate_Response__rosidl_typesupport_introspection_c__ComputeCoordinate_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t reseq_interfaces__srv__ComputeCoordinate_Response__rosidl_typesupport_introspection_c__ComputeCoordinate_Response_message_type_support_handle = {
  0,
  &reseq_interfaces__srv__ComputeCoordinate_Response__rosidl_typesupport_introspection_c__ComputeCoordinate_Response_message_members,
  get_message_typesupport_handle_function,
  &reseq_interfaces__srv__ComputeCoordinate_Response__get_type_hash,
  &reseq_interfaces__srv__ComputeCoordinate_Response__get_type_description,
  &reseq_interfaces__srv__ComputeCoordinate_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_reseq_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, reseq_interfaces, srv, ComputeCoordinate_Response)() {
  reseq_interfaces__srv__ComputeCoordinate_Response__rosidl_typesupport_introspection_c__ComputeCoordinate_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, PointStamped)();
  if (!reseq_interfaces__srv__ComputeCoordinate_Response__rosidl_typesupport_introspection_c__ComputeCoordinate_Response_message_type_support_handle.typesupport_identifier) {
    reseq_interfaces__srv__ComputeCoordinate_Response__rosidl_typesupport_introspection_c__ComputeCoordinate_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &reseq_interfaces__srv__ComputeCoordinate_Response__rosidl_typesupport_introspection_c__ComputeCoordinate_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "reseq_interfaces/srv/detail/compute_coordinate__rosidl_typesupport_introspection_c.h"
// already included above
// #include "reseq_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "reseq_interfaces/srv/detail/compute_coordinate__functions.h"
// already included above
// #include "reseq_interfaces/srv/detail/compute_coordinate__struct.h"


// Include directives for member types
// Member `info`
#include "service_msgs/msg/service_event_info.h"
// Member `info`
#include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
#include "reseq_interfaces/srv/compute_coordinate.h"
// Member `request`
// Member `response`
// already included above
// #include "reseq_interfaces/srv/detail/compute_coordinate__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void reseq_interfaces__srv__ComputeCoordinate_Event__rosidl_typesupport_introspection_c__ComputeCoordinate_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  reseq_interfaces__srv__ComputeCoordinate_Event__init(message_memory);
}

void reseq_interfaces__srv__ComputeCoordinate_Event__rosidl_typesupport_introspection_c__ComputeCoordinate_Event_fini_function(void * message_memory)
{
  reseq_interfaces__srv__ComputeCoordinate_Event__fini(message_memory);
}

size_t reseq_interfaces__srv__ComputeCoordinate_Event__rosidl_typesupport_introspection_c__size_function__ComputeCoordinate_Event__request(
  const void * untyped_member)
{
  const reseq_interfaces__srv__ComputeCoordinate_Request__Sequence * member =
    (const reseq_interfaces__srv__ComputeCoordinate_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * reseq_interfaces__srv__ComputeCoordinate_Event__rosidl_typesupport_introspection_c__get_const_function__ComputeCoordinate_Event__request(
  const void * untyped_member, size_t index)
{
  const reseq_interfaces__srv__ComputeCoordinate_Request__Sequence * member =
    (const reseq_interfaces__srv__ComputeCoordinate_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * reseq_interfaces__srv__ComputeCoordinate_Event__rosidl_typesupport_introspection_c__get_function__ComputeCoordinate_Event__request(
  void * untyped_member, size_t index)
{
  reseq_interfaces__srv__ComputeCoordinate_Request__Sequence * member =
    (reseq_interfaces__srv__ComputeCoordinate_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void reseq_interfaces__srv__ComputeCoordinate_Event__rosidl_typesupport_introspection_c__fetch_function__ComputeCoordinate_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const reseq_interfaces__srv__ComputeCoordinate_Request * item =
    ((const reseq_interfaces__srv__ComputeCoordinate_Request *)
    reseq_interfaces__srv__ComputeCoordinate_Event__rosidl_typesupport_introspection_c__get_const_function__ComputeCoordinate_Event__request(untyped_member, index));
  reseq_interfaces__srv__ComputeCoordinate_Request * value =
    (reseq_interfaces__srv__ComputeCoordinate_Request *)(untyped_value);
  *value = *item;
}

void reseq_interfaces__srv__ComputeCoordinate_Event__rosidl_typesupport_introspection_c__assign_function__ComputeCoordinate_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  reseq_interfaces__srv__ComputeCoordinate_Request * item =
    ((reseq_interfaces__srv__ComputeCoordinate_Request *)
    reseq_interfaces__srv__ComputeCoordinate_Event__rosidl_typesupport_introspection_c__get_function__ComputeCoordinate_Event__request(untyped_member, index));
  const reseq_interfaces__srv__ComputeCoordinate_Request * value =
    (const reseq_interfaces__srv__ComputeCoordinate_Request *)(untyped_value);
  *item = *value;
}

bool reseq_interfaces__srv__ComputeCoordinate_Event__rosidl_typesupport_introspection_c__resize_function__ComputeCoordinate_Event__request(
  void * untyped_member, size_t size)
{
  reseq_interfaces__srv__ComputeCoordinate_Request__Sequence * member =
    (reseq_interfaces__srv__ComputeCoordinate_Request__Sequence *)(untyped_member);
  reseq_interfaces__srv__ComputeCoordinate_Request__Sequence__fini(member);
  return reseq_interfaces__srv__ComputeCoordinate_Request__Sequence__init(member, size);
}

size_t reseq_interfaces__srv__ComputeCoordinate_Event__rosidl_typesupport_introspection_c__size_function__ComputeCoordinate_Event__response(
  const void * untyped_member)
{
  const reseq_interfaces__srv__ComputeCoordinate_Response__Sequence * member =
    (const reseq_interfaces__srv__ComputeCoordinate_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * reseq_interfaces__srv__ComputeCoordinate_Event__rosidl_typesupport_introspection_c__get_const_function__ComputeCoordinate_Event__response(
  const void * untyped_member, size_t index)
{
  const reseq_interfaces__srv__ComputeCoordinate_Response__Sequence * member =
    (const reseq_interfaces__srv__ComputeCoordinate_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * reseq_interfaces__srv__ComputeCoordinate_Event__rosidl_typesupport_introspection_c__get_function__ComputeCoordinate_Event__response(
  void * untyped_member, size_t index)
{
  reseq_interfaces__srv__ComputeCoordinate_Response__Sequence * member =
    (reseq_interfaces__srv__ComputeCoordinate_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void reseq_interfaces__srv__ComputeCoordinate_Event__rosidl_typesupport_introspection_c__fetch_function__ComputeCoordinate_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const reseq_interfaces__srv__ComputeCoordinate_Response * item =
    ((const reseq_interfaces__srv__ComputeCoordinate_Response *)
    reseq_interfaces__srv__ComputeCoordinate_Event__rosidl_typesupport_introspection_c__get_const_function__ComputeCoordinate_Event__response(untyped_member, index));
  reseq_interfaces__srv__ComputeCoordinate_Response * value =
    (reseq_interfaces__srv__ComputeCoordinate_Response *)(untyped_value);
  *value = *item;
}

void reseq_interfaces__srv__ComputeCoordinate_Event__rosidl_typesupport_introspection_c__assign_function__ComputeCoordinate_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  reseq_interfaces__srv__ComputeCoordinate_Response * item =
    ((reseq_interfaces__srv__ComputeCoordinate_Response *)
    reseq_interfaces__srv__ComputeCoordinate_Event__rosidl_typesupport_introspection_c__get_function__ComputeCoordinate_Event__response(untyped_member, index));
  const reseq_interfaces__srv__ComputeCoordinate_Response * value =
    (const reseq_interfaces__srv__ComputeCoordinate_Response *)(untyped_value);
  *item = *value;
}

bool reseq_interfaces__srv__ComputeCoordinate_Event__rosidl_typesupport_introspection_c__resize_function__ComputeCoordinate_Event__response(
  void * untyped_member, size_t size)
{
  reseq_interfaces__srv__ComputeCoordinate_Response__Sequence * member =
    (reseq_interfaces__srv__ComputeCoordinate_Response__Sequence *)(untyped_member);
  reseq_interfaces__srv__ComputeCoordinate_Response__Sequence__fini(member);
  return reseq_interfaces__srv__ComputeCoordinate_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember reseq_interfaces__srv__ComputeCoordinate_Event__rosidl_typesupport_introspection_c__ComputeCoordinate_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(reseq_interfaces__srv__ComputeCoordinate_Event, info),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "request",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(reseq_interfaces__srv__ComputeCoordinate_Event, request),  // bytes offset in struct
    NULL,  // default value
    reseq_interfaces__srv__ComputeCoordinate_Event__rosidl_typesupport_introspection_c__size_function__ComputeCoordinate_Event__request,  // size() function pointer
    reseq_interfaces__srv__ComputeCoordinate_Event__rosidl_typesupport_introspection_c__get_const_function__ComputeCoordinate_Event__request,  // get_const(index) function pointer
    reseq_interfaces__srv__ComputeCoordinate_Event__rosidl_typesupport_introspection_c__get_function__ComputeCoordinate_Event__request,  // get(index) function pointer
    reseq_interfaces__srv__ComputeCoordinate_Event__rosidl_typesupport_introspection_c__fetch_function__ComputeCoordinate_Event__request,  // fetch(index, &value) function pointer
    reseq_interfaces__srv__ComputeCoordinate_Event__rosidl_typesupport_introspection_c__assign_function__ComputeCoordinate_Event__request,  // assign(index, value) function pointer
    reseq_interfaces__srv__ComputeCoordinate_Event__rosidl_typesupport_introspection_c__resize_function__ComputeCoordinate_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(reseq_interfaces__srv__ComputeCoordinate_Event, response),  // bytes offset in struct
    NULL,  // default value
    reseq_interfaces__srv__ComputeCoordinate_Event__rosidl_typesupport_introspection_c__size_function__ComputeCoordinate_Event__response,  // size() function pointer
    reseq_interfaces__srv__ComputeCoordinate_Event__rosidl_typesupport_introspection_c__get_const_function__ComputeCoordinate_Event__response,  // get_const(index) function pointer
    reseq_interfaces__srv__ComputeCoordinate_Event__rosidl_typesupport_introspection_c__get_function__ComputeCoordinate_Event__response,  // get(index) function pointer
    reseq_interfaces__srv__ComputeCoordinate_Event__rosidl_typesupport_introspection_c__fetch_function__ComputeCoordinate_Event__response,  // fetch(index, &value) function pointer
    reseq_interfaces__srv__ComputeCoordinate_Event__rosidl_typesupport_introspection_c__assign_function__ComputeCoordinate_Event__response,  // assign(index, value) function pointer
    reseq_interfaces__srv__ComputeCoordinate_Event__rosidl_typesupport_introspection_c__resize_function__ComputeCoordinate_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers reseq_interfaces__srv__ComputeCoordinate_Event__rosidl_typesupport_introspection_c__ComputeCoordinate_Event_message_members = {
  "reseq_interfaces__srv",  // message namespace
  "ComputeCoordinate_Event",  // message name
  3,  // number of fields
  sizeof(reseq_interfaces__srv__ComputeCoordinate_Event),
  false,  // has_any_key_member_
  reseq_interfaces__srv__ComputeCoordinate_Event__rosidl_typesupport_introspection_c__ComputeCoordinate_Event_message_member_array,  // message members
  reseq_interfaces__srv__ComputeCoordinate_Event__rosidl_typesupport_introspection_c__ComputeCoordinate_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  reseq_interfaces__srv__ComputeCoordinate_Event__rosidl_typesupport_introspection_c__ComputeCoordinate_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t reseq_interfaces__srv__ComputeCoordinate_Event__rosidl_typesupport_introspection_c__ComputeCoordinate_Event_message_type_support_handle = {
  0,
  &reseq_interfaces__srv__ComputeCoordinate_Event__rosidl_typesupport_introspection_c__ComputeCoordinate_Event_message_members,
  get_message_typesupport_handle_function,
  &reseq_interfaces__srv__ComputeCoordinate_Event__get_type_hash,
  &reseq_interfaces__srv__ComputeCoordinate_Event__get_type_description,
  &reseq_interfaces__srv__ComputeCoordinate_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_reseq_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, reseq_interfaces, srv, ComputeCoordinate_Event)() {
  reseq_interfaces__srv__ComputeCoordinate_Event__rosidl_typesupport_introspection_c__ComputeCoordinate_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  reseq_interfaces__srv__ComputeCoordinate_Event__rosidl_typesupport_introspection_c__ComputeCoordinate_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, reseq_interfaces, srv, ComputeCoordinate_Request)();
  reseq_interfaces__srv__ComputeCoordinate_Event__rosidl_typesupport_introspection_c__ComputeCoordinate_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, reseq_interfaces, srv, ComputeCoordinate_Response)();
  if (!reseq_interfaces__srv__ComputeCoordinate_Event__rosidl_typesupport_introspection_c__ComputeCoordinate_Event_message_type_support_handle.typesupport_identifier) {
    reseq_interfaces__srv__ComputeCoordinate_Event__rosidl_typesupport_introspection_c__ComputeCoordinate_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &reseq_interfaces__srv__ComputeCoordinate_Event__rosidl_typesupport_introspection_c__ComputeCoordinate_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "reseq_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "reseq_interfaces/srv/detail/compute_coordinate__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers reseq_interfaces__srv__detail__compute_coordinate__rosidl_typesupport_introspection_c__ComputeCoordinate_service_members = {
  "reseq_interfaces__srv",  // service namespace
  "ComputeCoordinate",  // service name
  // the following fields are initialized below on first access
  NULL,  // request message
  // reseq_interfaces__srv__detail__compute_coordinate__rosidl_typesupport_introspection_c__ComputeCoordinate_Request_message_type_support_handle,
  NULL,  // response message
  // reseq_interfaces__srv__detail__compute_coordinate__rosidl_typesupport_introspection_c__ComputeCoordinate_Response_message_type_support_handle
  NULL  // event_message
  // reseq_interfaces__srv__detail__compute_coordinate__rosidl_typesupport_introspection_c__ComputeCoordinate_Response_message_type_support_handle
};


static rosidl_service_type_support_t reseq_interfaces__srv__detail__compute_coordinate__rosidl_typesupport_introspection_c__ComputeCoordinate_service_type_support_handle = {
  0,
  &reseq_interfaces__srv__detail__compute_coordinate__rosidl_typesupport_introspection_c__ComputeCoordinate_service_members,
  get_service_typesupport_handle_function,
  &reseq_interfaces__srv__ComputeCoordinate_Request__rosidl_typesupport_introspection_c__ComputeCoordinate_Request_message_type_support_handle,
  &reseq_interfaces__srv__ComputeCoordinate_Response__rosidl_typesupport_introspection_c__ComputeCoordinate_Response_message_type_support_handle,
  &reseq_interfaces__srv__ComputeCoordinate_Event__rosidl_typesupport_introspection_c__ComputeCoordinate_Event_message_type_support_handle,
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

// Forward declaration of message type support functions for service members
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, reseq_interfaces, srv, ComputeCoordinate_Request)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, reseq_interfaces, srv, ComputeCoordinate_Response)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, reseq_interfaces, srv, ComputeCoordinate_Event)(void);

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_reseq_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, reseq_interfaces, srv, ComputeCoordinate)(void) {
  if (!reseq_interfaces__srv__detail__compute_coordinate__rosidl_typesupport_introspection_c__ComputeCoordinate_service_type_support_handle.typesupport_identifier) {
    reseq_interfaces__srv__detail__compute_coordinate__rosidl_typesupport_introspection_c__ComputeCoordinate_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)reseq_interfaces__srv__detail__compute_coordinate__rosidl_typesupport_introspection_c__ComputeCoordinate_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, reseq_interfaces, srv, ComputeCoordinate_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, reseq_interfaces, srv, ComputeCoordinate_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, reseq_interfaces, srv, ComputeCoordinate_Event)()->data;
  }

  return &reseq_interfaces__srv__detail__compute_coordinate__rosidl_typesupport_introspection_c__ComputeCoordinate_service_type_support_handle;
}
