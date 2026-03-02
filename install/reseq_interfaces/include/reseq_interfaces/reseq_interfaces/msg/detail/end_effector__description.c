// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from reseq_interfaces:msg/EndEffector.idl
// generated code does not contain a copyright notice

#include "reseq_interfaces/msg/detail/end_effector__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
const rosidl_type_hash_t *
reseq_interfaces__msg__EndEffector__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xbf, 0x5e, 0x75, 0xb5, 0x19, 0x3b, 0x79, 0x61,
      0x6c, 0xb5, 0x6b, 0x5f, 0x93, 0x43, 0xe3, 0x0e,
      0x0c, 0xb6, 0x29, 0x90, 0x6b, 0x28, 0x60, 0xe4,
      0x5e, 0x5e, 0x5c, 0xf0, 0x2c, 0xf3, 0xb0, 0x30,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char reseq_interfaces__msg__EndEffector__TYPE_NAME[] = "reseq_interfaces/msg/EndEffector";

// Define type names, field names, and default values
static char reseq_interfaces__msg__EndEffector__FIELD_NAME__pitch_vel[] = "pitch_vel";
static char reseq_interfaces__msg__EndEffector__FIELD_NAME__head_pitch_vel[] = "head_pitch_vel";
static char reseq_interfaces__msg__EndEffector__FIELD_NAME__head_roll_vel[] = "head_roll_vel";

static rosidl_runtime_c__type_description__Field reseq_interfaces__msg__EndEffector__FIELDS[] = {
  {
    {reseq_interfaces__msg__EndEffector__FIELD_NAME__pitch_vel, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {reseq_interfaces__msg__EndEffector__FIELD_NAME__head_pitch_vel, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {reseq_interfaces__msg__EndEffector__FIELD_NAME__head_roll_vel, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
reseq_interfaces__msg__EndEffector__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {reseq_interfaces__msg__EndEffector__TYPE_NAME, 32, 32},
      {reseq_interfaces__msg__EndEffector__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float32 pitch_vel\n"
  "float32 head_pitch_vel\n"
  "float32 head_roll_vel";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
reseq_interfaces__msg__EndEffector__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {reseq_interfaces__msg__EndEffector__TYPE_NAME, 32, 32},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 63, 63},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
reseq_interfaces__msg__EndEffector__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *reseq_interfaces__msg__EndEffector__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
