// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from reseq_interfaces:msg/Remote.idl
// generated code does not contain a copyright notice

#include "reseq_interfaces/msg/detail/remote__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
const rosidl_type_hash_t *
reseq_interfaces__msg__Remote__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x75, 0x17, 0x95, 0xc7, 0xd2, 0x9d, 0x9d, 0x6b,
      0xd3, 0x32, 0xd0, 0x22, 0x52, 0x56, 0x0b, 0xfe,
      0xc9, 0x8b, 0x98, 0x39, 0xbc, 0xf0, 0xb6, 0xf8,
      0x26, 0x70, 0xbc, 0x94, 0x4c, 0xdc, 0xb4, 0xd4,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "geometry_msgs/msg/detail/vector3__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t geometry_msgs__msg__Vector3__EXPECTED_HASH = {1, {
    0xcc, 0x12, 0xfe, 0x83, 0xe4, 0xc0, 0x27, 0x19,
    0xf1, 0xce, 0x80, 0x70, 0xbf, 0xd1, 0x4a, 0xec,
    0xd4, 0x0f, 0x75, 0xa9, 0x66, 0x96, 0xa6, 0x7a,
    0x2a, 0x1f, 0x37, 0xf7, 0xdb, 0xb0, 0x76, 0x5d,
  }};
#endif

static char reseq_interfaces__msg__Remote__TYPE_NAME[] = "reseq_interfaces/msg/Remote";
static char geometry_msgs__msg__Vector3__TYPE_NAME[] = "geometry_msgs/msg/Vector3";

// Define type names, field names, and default values
static char reseq_interfaces__msg__Remote__FIELD_NAME__left[] = "left";
static char reseq_interfaces__msg__Remote__FIELD_NAME__right[] = "right";
static char reseq_interfaces__msg__Remote__FIELD_NAME__buttons[] = "buttons";
static char reseq_interfaces__msg__Remote__FIELD_NAME__switches[] = "switches";

static rosidl_runtime_c__type_description__Field reseq_interfaces__msg__Remote__FIELDS[] = {
  {
    {reseq_interfaces__msg__Remote__FIELD_NAME__left, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__Vector3__TYPE_NAME, 25, 25},
    },
    {NULL, 0, 0},
  },
  {
    {reseq_interfaces__msg__Remote__FIELD_NAME__right, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__Vector3__TYPE_NAME, 25, 25},
    },
    {NULL, 0, 0},
  },
  {
    {reseq_interfaces__msg__Remote__FIELD_NAME__buttons, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {reseq_interfaces__msg__Remote__FIELD_NAME__switches, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription reseq_interfaces__msg__Remote__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {geometry_msgs__msg__Vector3__TYPE_NAME, 25, 25},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
reseq_interfaces__msg__Remote__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {reseq_interfaces__msg__Remote__TYPE_NAME, 27, 27},
      {reseq_interfaces__msg__Remote__FIELDS, 4, 4},
    },
    {reseq_interfaces__msg__Remote__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&geometry_msgs__msg__Vector3__EXPECTED_HASH, geometry_msgs__msg__Vector3__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = geometry_msgs__msg__Vector3__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Input from 3-axes joysticks, buttons and switches.\n"
  "\n"
  "geometry_msgs/Vector3 left\n"
  "geometry_msgs/Vector3 right\n"
  "bool[] buttons\n"
  "bool[] switches";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
reseq_interfaces__msg__Remote__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {reseq_interfaces__msg__Remote__TYPE_NAME, 27, 27},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 140, 140},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
reseq_interfaces__msg__Remote__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *reseq_interfaces__msg__Remote__get_individual_type_description_source(NULL),
    sources[1] = *geometry_msgs__msg__Vector3__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
