// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from reseq_interfaces:srv/SetMode.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "reseq_interfaces/srv/set_mode.h"


#ifndef RESEQ_INTERFACES__SRV__DETAIL__SET_MODE__STRUCT_H_
#define RESEQ_INTERFACES__SRV__DETAIL__SET_MODE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'csv_path'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/SetMode in the package reseq_interfaces.
typedef struct reseq_interfaces__srv__SetMode_Request
{
  /// 0=OFF,1=INITIALIZING,2=SENSOR_CRATE,3=MAPPING
  uint8_t mode;
  rosidl_runtime_c__String csv_path;
} reseq_interfaces__srv__SetMode_Request;

// Struct for a sequence of reseq_interfaces__srv__SetMode_Request.
typedef struct reseq_interfaces__srv__SetMode_Request__Sequence
{
  reseq_interfaces__srv__SetMode_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} reseq_interfaces__srv__SetMode_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/SetMode in the package reseq_interfaces.
typedef struct reseq_interfaces__srv__SetMode_Response
{
  bool success;
  uint8_t previous_mode;
  rosidl_runtime_c__String message;
} reseq_interfaces__srv__SetMode_Response;

// Struct for a sequence of reseq_interfaces__srv__SetMode_Response.
typedef struct reseq_interfaces__srv__SetMode_Response__Sequence
{
  reseq_interfaces__srv__SetMode_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} reseq_interfaces__srv__SetMode_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  reseq_interfaces__srv__SetMode_Event__request__MAX_SIZE = 1
};
// response
enum
{
  reseq_interfaces__srv__SetMode_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/SetMode in the package reseq_interfaces.
typedef struct reseq_interfaces__srv__SetMode_Event
{
  service_msgs__msg__ServiceEventInfo info;
  reseq_interfaces__srv__SetMode_Request__Sequence request;
  reseq_interfaces__srv__SetMode_Response__Sequence response;
} reseq_interfaces__srv__SetMode_Event;

// Struct for a sequence of reseq_interfaces__srv__SetMode_Event.
typedef struct reseq_interfaces__srv__SetMode_Event__Sequence
{
  reseq_interfaces__srv__SetMode_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} reseq_interfaces__srv__SetMode_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RESEQ_INTERFACES__SRV__DETAIL__SET_MODE__STRUCT_H_
