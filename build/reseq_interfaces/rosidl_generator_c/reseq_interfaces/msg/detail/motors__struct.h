// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from reseq_interfaces:msg/Motors.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "reseq_interfaces/msg/motors.h"


#ifndef RESEQ_INTERFACES__MSG__DETAIL__MOTORS__STRUCT_H_
#define RESEQ_INTERFACES__MSG__DETAIL__MOTORS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/Motors in the package reseq_interfaces.
/**
  * Speed of the left and right motors of a single module
 */
typedef struct reseq_interfaces__msg__Motors
{
  float left;
  float right;
} reseq_interfaces__msg__Motors;

// Struct for a sequence of reseq_interfaces__msg__Motors.
typedef struct reseq_interfaces__msg__Motors__Sequence
{
  reseq_interfaces__msg__Motors * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} reseq_interfaces__msg__Motors__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RESEQ_INTERFACES__MSG__DETAIL__MOTORS__STRUCT_H_
