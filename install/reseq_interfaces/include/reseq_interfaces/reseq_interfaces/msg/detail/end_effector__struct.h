// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from reseq_interfaces:msg/EndEffector.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "reseq_interfaces/msg/end_effector.h"


#ifndef RESEQ_INTERFACES__MSG__DETAIL__END_EFFECTOR__STRUCT_H_
#define RESEQ_INTERFACES__MSG__DETAIL__END_EFFECTOR__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/EndEffector in the package reseq_interfaces.
typedef struct reseq_interfaces__msg__EndEffector
{
  float pitch_vel;
  float head_pitch_vel;
  float head_roll_vel;
} reseq_interfaces__msg__EndEffector;

// Struct for a sequence of reseq_interfaces__msg__EndEffector.
typedef struct reseq_interfaces__msg__EndEffector__Sequence
{
  reseq_interfaces__msg__EndEffector * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} reseq_interfaces__msg__EndEffector__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RESEQ_INTERFACES__MSG__DETAIL__END_EFFECTOR__STRUCT_H_
