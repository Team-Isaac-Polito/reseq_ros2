// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from reseq_interfaces:srv/BatchDetections2D.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "reseq_interfaces/srv/batch_detections2_d.h"


#ifndef RESEQ_INTERFACES__SRV__DETAIL__BATCH_DETECTIONS2_D__STRUCT_H_
#define RESEQ_INTERFACES__SRV__DETAIL__BATCH_DETECTIONS2_D__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'robot'
// Member 'mode'
#include "rosidl_runtime_c/string.h"
// Member 'detections'
#include "vision_msgs/msg/detail/detection2_d_array__struct.h"

/// Struct defined in srv/BatchDetections2D in the package reseq_interfaces.
typedef struct reseq_interfaces__srv__BatchDetections2D_Request
{
  rosidl_runtime_c__String robot;
  /// 'A' or 'T'
  rosidl_runtime_c__String mode;
  vision_msgs__msg__Detection2DArray detections;
} reseq_interfaces__srv__BatchDetections2D_Request;

// Struct for a sequence of reseq_interfaces__srv__BatchDetections2D_Request.
typedef struct reseq_interfaces__srv__BatchDetections2D_Request__Sequence
{
  reseq_interfaces__srv__BatchDetections2D_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} reseq_interfaces__srv__BatchDetections2D_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/BatchDetections2D in the package reseq_interfaces.
typedef struct reseq_interfaces__srv__BatchDetections2D_Response
{
  bool success;
  /// filepath or error
  rosidl_runtime_c__String message;
} reseq_interfaces__srv__BatchDetections2D_Response;

// Struct for a sequence of reseq_interfaces__srv__BatchDetections2D_Response.
typedef struct reseq_interfaces__srv__BatchDetections2D_Response__Sequence
{
  reseq_interfaces__srv__BatchDetections2D_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} reseq_interfaces__srv__BatchDetections2D_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  reseq_interfaces__srv__BatchDetections2D_Event__request__MAX_SIZE = 1
};
// response
enum
{
  reseq_interfaces__srv__BatchDetections2D_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/BatchDetections2D in the package reseq_interfaces.
typedef struct reseq_interfaces__srv__BatchDetections2D_Event
{
  service_msgs__msg__ServiceEventInfo info;
  reseq_interfaces__srv__BatchDetections2D_Request__Sequence request;
  reseq_interfaces__srv__BatchDetections2D_Response__Sequence response;
} reseq_interfaces__srv__BatchDetections2D_Event;

// Struct for a sequence of reseq_interfaces__srv__BatchDetections2D_Event.
typedef struct reseq_interfaces__srv__BatchDetections2D_Event__Sequence
{
  reseq_interfaces__srv__BatchDetections2D_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} reseq_interfaces__srv__BatchDetections2D_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RESEQ_INTERFACES__SRV__DETAIL__BATCH_DETECTIONS2_D__STRUCT_H_
