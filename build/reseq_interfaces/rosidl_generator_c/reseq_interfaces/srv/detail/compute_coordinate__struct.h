// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from reseq_interfaces:srv/ComputeCoordinate.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "reseq_interfaces/srv/compute_coordinate.h"


#ifndef RESEQ_INTERFACES__SRV__DETAIL__COMPUTE_COORDINATE__STRUCT_H_
#define RESEQ_INTERFACES__SRV__DETAIL__COMPUTE_COORDINATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'detection'
#include "reseq_interfaces/msg/detail/detection__struct.h"
// Member 'camera_info'
#include "sensor_msgs/msg/detail/camera_info__struct.h"
// Member 'target_frame'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/ComputeCoordinate in the package reseq_interfaces.
typedef struct reseq_interfaces__srv__ComputeCoordinate_Request
{
  reseq_interfaces__msg__Detection detection;
  sensor_msgs__msg__CameraInfo camera_info;
  rosidl_runtime_c__String target_frame;
} reseq_interfaces__srv__ComputeCoordinate_Request;

// Struct for a sequence of reseq_interfaces__srv__ComputeCoordinate_Request.
typedef struct reseq_interfaces__srv__ComputeCoordinate_Request__Sequence
{
  reseq_interfaces__srv__ComputeCoordinate_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} reseq_interfaces__srv__ComputeCoordinate_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'point'
#include "geometry_msgs/msg/detail/point_stamped__struct.h"
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/ComputeCoordinate in the package reseq_interfaces.
typedef struct reseq_interfaces__srv__ComputeCoordinate_Response
{
  bool success;
  geometry_msgs__msg__PointStamped point;
  rosidl_runtime_c__String message;
} reseq_interfaces__srv__ComputeCoordinate_Response;

// Struct for a sequence of reseq_interfaces__srv__ComputeCoordinate_Response.
typedef struct reseq_interfaces__srv__ComputeCoordinate_Response__Sequence
{
  reseq_interfaces__srv__ComputeCoordinate_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} reseq_interfaces__srv__ComputeCoordinate_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  reseq_interfaces__srv__ComputeCoordinate_Event__request__MAX_SIZE = 1
};
// response
enum
{
  reseq_interfaces__srv__ComputeCoordinate_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/ComputeCoordinate in the package reseq_interfaces.
typedef struct reseq_interfaces__srv__ComputeCoordinate_Event
{
  service_msgs__msg__ServiceEventInfo info;
  reseq_interfaces__srv__ComputeCoordinate_Request__Sequence request;
  reseq_interfaces__srv__ComputeCoordinate_Response__Sequence response;
} reseq_interfaces__srv__ComputeCoordinate_Event;

// Struct for a sequence of reseq_interfaces__srv__ComputeCoordinate_Event.
typedef struct reseq_interfaces__srv__ComputeCoordinate_Event__Sequence
{
  reseq_interfaces__srv__ComputeCoordinate_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} reseq_interfaces__srv__ComputeCoordinate_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RESEQ_INTERFACES__SRV__DETAIL__COMPUTE_COORDINATE__STRUCT_H_
