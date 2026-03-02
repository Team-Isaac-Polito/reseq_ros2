// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from reseq_interfaces:msg/Motors.idl
// generated code does not contain a copyright notice

#include "reseq_interfaces/msg/detail/motors__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
const rosidl_type_hash_t *
reseq_interfaces__msg__Motors__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xa8, 0x98, 0x7e, 0x18, 0x4d, 0x98, 0x22, 0x0d,
      0x8b, 0x5f, 0x0f, 0x50, 0x75, 0xc1, 0xb1, 0xbe,
      0x29, 0xd0, 0x4a, 0xb7, 0xe8, 0x69, 0xbc, 0x6b,
      0x3f, 0xa1, 0x5e, 0x32, 0xe2, 0x23, 0xf9, 0xf1,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char reseq_interfaces__msg__Motors__TYPE_NAME[] = "reseq_interfaces/msg/Motors";

// Define type names, field names, and default values
static char reseq_interfaces__msg__Motors__FIELD_NAME__left[] = "left";
static char reseq_interfaces__msg__Motors__FIELD_NAME__right[] = "right";

static rosidl_runtime_c__type_description__Field reseq_interfaces__msg__Motors__FIELDS[] = {
  {
    {reseq_interfaces__msg__Motors__FIELD_NAME__left, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {reseq_interfaces__msg__Motors__FIELD_NAME__right, 5, 5},
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
reseq_interfaces__msg__Motors__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {reseq_interfaces__msg__Motors__TYPE_NAME, 27, 27},
      {reseq_interfaces__msg__Motors__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Speed of the left and right motors of a single module\n"
  "\n"
  "float32 left\n"
  "float32 right";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
reseq_interfaces__msg__Motors__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {reseq_interfaces__msg__Motors__TYPE_NAME, 27, 27},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 84, 84},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
reseq_interfaces__msg__Motors__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *reseq_interfaces__msg__Motors__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
