// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from reseq_interfaces:msg/Remote.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "reseq_interfaces/msg/remote.h"


#ifndef RESEQ_INTERFACES__MSG__DETAIL__REMOTE__STRUCT_H_
#define RESEQ_INTERFACES__MSG__DETAIL__REMOTE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'left'
// Member 'right'
#include "geometry_msgs/msg/detail/vector3__struct.h"
// Member 'buttons'
// Member 'switches'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/Remote in the package reseq_interfaces.
/**
  * Input from 3-axes joysticks, buttons and switches.
 */
typedef struct reseq_interfaces__msg__Remote
{
  geometry_msgs__msg__Vector3 left;
  geometry_msgs__msg__Vector3 right;
  rosidl_runtime_c__boolean__Sequence buttons;
  rosidl_runtime_c__boolean__Sequence switches;
} reseq_interfaces__msg__Remote;

// Struct for a sequence of reseq_interfaces__msg__Remote.
typedef struct reseq_interfaces__msg__Remote__Sequence
{
  reseq_interfaces__msg__Remote * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} reseq_interfaces__msg__Remote__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RESEQ_INTERFACES__MSG__DETAIL__REMOTE__STRUCT_H_
