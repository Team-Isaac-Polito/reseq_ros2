// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from reseq_interfaces:srv/GetStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "reseq_interfaces/srv/get_status.h"


#ifndef RESEQ_INTERFACES__SRV__DETAIL__GET_STATUS__STRUCT_H_
#define RESEQ_INTERFACES__SRV__DETAIL__GET_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/GetStatus in the package reseq_interfaces.
typedef struct reseq_interfaces__srv__GetStatus_Request
{
  uint8_t structure_needs_at_least_one_member;
} reseq_interfaces__srv__GetStatus_Request;

// Struct for a sequence of reseq_interfaces__srv__GetStatus_Request.
typedef struct reseq_interfaces__srv__GetStatus_Request__Sequence
{
  reseq_interfaces__srv__GetStatus_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} reseq_interfaces__srv__GetStatus_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'csv_path'
// Member 'last_error'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/GetStatus in the package reseq_interfaces.
typedef struct reseq_interfaces__srv__GetStatus_Response
{
  uint8_t current_mode;
  bool initialized;
  rosidl_runtime_c__String csv_path;
  rosidl_runtime_c__String last_error;
} reseq_interfaces__srv__GetStatus_Response;

// Struct for a sequence of reseq_interfaces__srv__GetStatus_Response.
typedef struct reseq_interfaces__srv__GetStatus_Response__Sequence
{
  reseq_interfaces__srv__GetStatus_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} reseq_interfaces__srv__GetStatus_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  reseq_interfaces__srv__GetStatus_Event__request__MAX_SIZE = 1
};
// response
enum
{
  reseq_interfaces__srv__GetStatus_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/GetStatus in the package reseq_interfaces.
typedef struct reseq_interfaces__srv__GetStatus_Event
{
  service_msgs__msg__ServiceEventInfo info;
  reseq_interfaces__srv__GetStatus_Request__Sequence request;
  reseq_interfaces__srv__GetStatus_Response__Sequence response;
} reseq_interfaces__srv__GetStatus_Event;

// Struct for a sequence of reseq_interfaces__srv__GetStatus_Event.
typedef struct reseq_interfaces__srv__GetStatus_Event__Sequence
{
  reseq_interfaces__srv__GetStatus_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} reseq_interfaces__srv__GetStatus_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RESEQ_INTERFACES__SRV__DETAIL__GET_STATUS__STRUCT_H_
