// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from reseq_interfaces:msg/Detection.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "reseq_interfaces/msg/detection.h"


#ifndef RESEQ_INTERFACES__MSG__DETAIL__DETECTION__STRUCT_H_
#define RESEQ_INTERFACES__MSG__DETAIL__DETECTION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'type'
// Member 'name'
// Member 'robot'
// Member 'mode'
// Member 'camera_frame'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Detection in the package reseq_interfaces.
typedef struct reseq_interfaces__msg__Detection
{
  /// stamp + frame_id (camera frame)
  std_msgs__msg__Header header;
  int32_t detection;
  rosidl_runtime_c__String type;
  rosidl_runtime_c__String name;
  rosidl_runtime_c__String robot;
  rosidl_runtime_c__String mode;
  float confidence;
  int32_t xmin;
  int32_t ymin;
  uint32_t width;
  uint32_t height;
  /// meters (depth at bounding-box center)
  float depth_center;
  rosidl_runtime_c__String camera_frame;
} reseq_interfaces__msg__Detection;

// Struct for a sequence of reseq_interfaces__msg__Detection.
typedef struct reseq_interfaces__msg__Detection__Sequence
{
  reseq_interfaces__msg__Detection * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} reseq_interfaces__msg__Detection__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RESEQ_INTERFACES__MSG__DETAIL__DETECTION__STRUCT_H_
