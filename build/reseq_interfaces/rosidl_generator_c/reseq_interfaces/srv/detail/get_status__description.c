// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from reseq_interfaces:srv/GetStatus.idl
// generated code does not contain a copyright notice

#include "reseq_interfaces/srv/detail/get_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
const rosidl_type_hash_t *
reseq_interfaces__srv__GetStatus__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xf4, 0xa0, 0x2d, 0x59, 0xee, 0xe3, 0x5a, 0x7e,
      0x6c, 0x94, 0xf3, 0xd2, 0x39, 0x14, 0xe5, 0x6f,
      0x58, 0x6d, 0x68, 0xaa, 0x5c, 0x1e, 0x03, 0x8f,
      0xdc, 0x07, 0xc9, 0x75, 0x12, 0x32, 0xe5, 0x47,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
const rosidl_type_hash_t *
reseq_interfaces__srv__GetStatus_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xe1, 0x62, 0xc9, 0x4a, 0x4f, 0x4d, 0x2a, 0x2d,
      0xbc, 0x68, 0x5e, 0x82, 0x8e, 0x3f, 0x66, 0x6a,
      0x95, 0x50, 0x16, 0x29, 0x8a, 0x33, 0x43, 0xe5,
      0xbd, 0x40, 0x58, 0x98, 0xc8, 0x48, 0x5c, 0xe7,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
const rosidl_type_hash_t *
reseq_interfaces__srv__GetStatus_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x4e, 0x06, 0xe2, 0xf3, 0x5c, 0xb4, 0xbd, 0x40,
      0x83, 0x8b, 0xa7, 0x89, 0x0f, 0x9c, 0x82, 0xb5,
      0x5b, 0xd3, 0x5c, 0x14, 0x78, 0x69, 0x75, 0xcc,
      0x4e, 0x9c, 0xf0, 0x30, 0x4e, 0xb1, 0x06, 0x75,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
const rosidl_type_hash_t *
reseq_interfaces__srv__GetStatus_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x60, 0xf5, 0x71, 0x05, 0x98, 0x3b, 0xa3, 0x77,
      0xce, 0x24, 0xc4, 0x9a, 0x96, 0x43, 0x3a, 0x9e,
      0x8a, 0x60, 0xec, 0xdd, 0x9a, 0xe0, 0x78, 0x3f,
      0x35, 0x50, 0x50, 0x2b, 0xff, 0xd7, 0xcd, 0xff,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "service_msgs/msg/detail/service_event_info__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
#endif

static char reseq_interfaces__srv__GetStatus__TYPE_NAME[] = "reseq_interfaces/srv/GetStatus";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char reseq_interfaces__srv__GetStatus_Event__TYPE_NAME[] = "reseq_interfaces/srv/GetStatus_Event";
static char reseq_interfaces__srv__GetStatus_Request__TYPE_NAME[] = "reseq_interfaces/srv/GetStatus_Request";
static char reseq_interfaces__srv__GetStatus_Response__TYPE_NAME[] = "reseq_interfaces/srv/GetStatus_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char reseq_interfaces__srv__GetStatus__FIELD_NAME__request_message[] = "request_message";
static char reseq_interfaces__srv__GetStatus__FIELD_NAME__response_message[] = "response_message";
static char reseq_interfaces__srv__GetStatus__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field reseq_interfaces__srv__GetStatus__FIELDS[] = {
  {
    {reseq_interfaces__srv__GetStatus__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {reseq_interfaces__srv__GetStatus_Request__TYPE_NAME, 38, 38},
    },
    {NULL, 0, 0},
  },
  {
    {reseq_interfaces__srv__GetStatus__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {reseq_interfaces__srv__GetStatus_Response__TYPE_NAME, 39, 39},
    },
    {NULL, 0, 0},
  },
  {
    {reseq_interfaces__srv__GetStatus__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {reseq_interfaces__srv__GetStatus_Event__TYPE_NAME, 36, 36},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription reseq_interfaces__srv__GetStatus__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {reseq_interfaces__srv__GetStatus_Event__TYPE_NAME, 36, 36},
    {NULL, 0, 0},
  },
  {
    {reseq_interfaces__srv__GetStatus_Request__TYPE_NAME, 38, 38},
    {NULL, 0, 0},
  },
  {
    {reseq_interfaces__srv__GetStatus_Response__TYPE_NAME, 39, 39},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
reseq_interfaces__srv__GetStatus__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {reseq_interfaces__srv__GetStatus__TYPE_NAME, 30, 30},
      {reseq_interfaces__srv__GetStatus__FIELDS, 3, 3},
    },
    {reseq_interfaces__srv__GetStatus__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = reseq_interfaces__srv__GetStatus_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = reseq_interfaces__srv__GetStatus_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = reseq_interfaces__srv__GetStatus_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char reseq_interfaces__srv__GetStatus_Request__FIELD_NAME__structure_needs_at_least_one_member[] = "structure_needs_at_least_one_member";

static rosidl_runtime_c__type_description__Field reseq_interfaces__srv__GetStatus_Request__FIELDS[] = {
  {
    {reseq_interfaces__srv__GetStatus_Request__FIELD_NAME__structure_needs_at_least_one_member, 35, 35},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
reseq_interfaces__srv__GetStatus_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {reseq_interfaces__srv__GetStatus_Request__TYPE_NAME, 38, 38},
      {reseq_interfaces__srv__GetStatus_Request__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char reseq_interfaces__srv__GetStatus_Response__FIELD_NAME__current_mode[] = "current_mode";
static char reseq_interfaces__srv__GetStatus_Response__FIELD_NAME__initialized[] = "initialized";
static char reseq_interfaces__srv__GetStatus_Response__FIELD_NAME__csv_path[] = "csv_path";
static char reseq_interfaces__srv__GetStatus_Response__FIELD_NAME__last_error[] = "last_error";

static rosidl_runtime_c__type_description__Field reseq_interfaces__srv__GetStatus_Response__FIELDS[] = {
  {
    {reseq_interfaces__srv__GetStatus_Response__FIELD_NAME__current_mode, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {reseq_interfaces__srv__GetStatus_Response__FIELD_NAME__initialized, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {reseq_interfaces__srv__GetStatus_Response__FIELD_NAME__csv_path, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {reseq_interfaces__srv__GetStatus_Response__FIELD_NAME__last_error, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
reseq_interfaces__srv__GetStatus_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {reseq_interfaces__srv__GetStatus_Response__TYPE_NAME, 39, 39},
      {reseq_interfaces__srv__GetStatus_Response__FIELDS, 4, 4},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char reseq_interfaces__srv__GetStatus_Event__FIELD_NAME__info[] = "info";
static char reseq_interfaces__srv__GetStatus_Event__FIELD_NAME__request[] = "request";
static char reseq_interfaces__srv__GetStatus_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field reseq_interfaces__srv__GetStatus_Event__FIELDS[] = {
  {
    {reseq_interfaces__srv__GetStatus_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {reseq_interfaces__srv__GetStatus_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {reseq_interfaces__srv__GetStatus_Request__TYPE_NAME, 38, 38},
    },
    {NULL, 0, 0},
  },
  {
    {reseq_interfaces__srv__GetStatus_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {reseq_interfaces__srv__GetStatus_Response__TYPE_NAME, 39, 39},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription reseq_interfaces__srv__GetStatus_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {reseq_interfaces__srv__GetStatus_Request__TYPE_NAME, 38, 38},
    {NULL, 0, 0},
  },
  {
    {reseq_interfaces__srv__GetStatus_Response__TYPE_NAME, 39, 39},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
reseq_interfaces__srv__GetStatus_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {reseq_interfaces__srv__GetStatus_Event__TYPE_NAME, 36, 36},
      {reseq_interfaces__srv__GetStatus_Event__FIELDS, 3, 3},
    },
    {reseq_interfaces__srv__GetStatus_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = reseq_interfaces__srv__GetStatus_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = reseq_interfaces__srv__GetStatus_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Request\n"
  "# (empty)\n"
  "---\n"
  "uint8 current_mode\n"
  "bool initialized\n"
  "string csv_path\n"
  "string last_error";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
reseq_interfaces__srv__GetStatus__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {reseq_interfaces__srv__GetStatus__TYPE_NAME, 30, 30},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 93, 93},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
reseq_interfaces__srv__GetStatus_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {reseq_interfaces__srv__GetStatus_Request__TYPE_NAME, 38, 38},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
reseq_interfaces__srv__GetStatus_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {reseq_interfaces__srv__GetStatus_Response__TYPE_NAME, 39, 39},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
reseq_interfaces__srv__GetStatus_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {reseq_interfaces__srv__GetStatus_Event__TYPE_NAME, 36, 36},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
reseq_interfaces__srv__GetStatus__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *reseq_interfaces__srv__GetStatus__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *reseq_interfaces__srv__GetStatus_Event__get_individual_type_description_source(NULL);
    sources[3] = *reseq_interfaces__srv__GetStatus_Request__get_individual_type_description_source(NULL);
    sources[4] = *reseq_interfaces__srv__GetStatus_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
reseq_interfaces__srv__GetStatus_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *reseq_interfaces__srv__GetStatus_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
reseq_interfaces__srv__GetStatus_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *reseq_interfaces__srv__GetStatus_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
reseq_interfaces__srv__GetStatus_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *reseq_interfaces__srv__GetStatus_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *reseq_interfaces__srv__GetStatus_Request__get_individual_type_description_source(NULL);
    sources[3] = *reseq_interfaces__srv__GetStatus_Response__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
