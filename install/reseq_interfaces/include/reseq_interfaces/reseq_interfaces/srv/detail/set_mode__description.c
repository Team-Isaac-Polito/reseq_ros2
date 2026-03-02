// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from reseq_interfaces:srv/SetMode.idl
// generated code does not contain a copyright notice

#include "reseq_interfaces/srv/detail/set_mode__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
const rosidl_type_hash_t *
reseq_interfaces__srv__SetMode__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xa5, 0x5a, 0xaa, 0x37, 0x54, 0xa8, 0x6f, 0xe9,
      0x27, 0xd2, 0x31, 0x4e, 0xf7, 0x1a, 0x35, 0xef,
      0x02, 0xb5, 0x45, 0x8c, 0x50, 0xb4, 0xcf, 0x32,
      0x84, 0x12, 0x3f, 0xa7, 0x32, 0x4d, 0x8b, 0x03,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
const rosidl_type_hash_t *
reseq_interfaces__srv__SetMode_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x0d, 0xde, 0xa0, 0xb5, 0x63, 0x25, 0x9d, 0x90,
      0xe0, 0x2d, 0x38, 0x2e, 0x03, 0x63, 0x42, 0x3f,
      0x79, 0x84, 0xf2, 0x31, 0x68, 0xfd, 0xcc, 0x06,
      0xdb, 0xc2, 0x67, 0xc2, 0x5a, 0x8c, 0x69, 0x86,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
const rosidl_type_hash_t *
reseq_interfaces__srv__SetMode_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x4d, 0xdc, 0x1d, 0x05, 0x91, 0x3f, 0x23, 0x90,
      0x40, 0x98, 0xf7, 0x3b, 0x65, 0xa9, 0xc9, 0x1e,
      0x47, 0x0d, 0xef, 0xb4, 0x5c, 0x23, 0x70, 0x99,
      0xe2, 0x9f, 0xa2, 0x2c, 0xb0, 0xe1, 0xbb, 0x7b,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
const rosidl_type_hash_t *
reseq_interfaces__srv__SetMode_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xb0, 0xf2, 0xc5, 0x15, 0xbe, 0xd9, 0xac, 0x4f,
      0xff, 0x2b, 0x74, 0x9a, 0x42, 0x0f, 0x20, 0xff,
      0xde, 0xe4, 0x43, 0x63, 0x1d, 0x9e, 0xfd, 0xd7,
      0x5e, 0xb3, 0x86, 0xd4, 0xfd, 0xbd, 0x2a, 0xe8,
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

static char reseq_interfaces__srv__SetMode__TYPE_NAME[] = "reseq_interfaces/srv/SetMode";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char reseq_interfaces__srv__SetMode_Event__TYPE_NAME[] = "reseq_interfaces/srv/SetMode_Event";
static char reseq_interfaces__srv__SetMode_Request__TYPE_NAME[] = "reseq_interfaces/srv/SetMode_Request";
static char reseq_interfaces__srv__SetMode_Response__TYPE_NAME[] = "reseq_interfaces/srv/SetMode_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char reseq_interfaces__srv__SetMode__FIELD_NAME__request_message[] = "request_message";
static char reseq_interfaces__srv__SetMode__FIELD_NAME__response_message[] = "response_message";
static char reseq_interfaces__srv__SetMode__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field reseq_interfaces__srv__SetMode__FIELDS[] = {
  {
    {reseq_interfaces__srv__SetMode__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {reseq_interfaces__srv__SetMode_Request__TYPE_NAME, 36, 36},
    },
    {NULL, 0, 0},
  },
  {
    {reseq_interfaces__srv__SetMode__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {reseq_interfaces__srv__SetMode_Response__TYPE_NAME, 37, 37},
    },
    {NULL, 0, 0},
  },
  {
    {reseq_interfaces__srv__SetMode__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {reseq_interfaces__srv__SetMode_Event__TYPE_NAME, 34, 34},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription reseq_interfaces__srv__SetMode__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {reseq_interfaces__srv__SetMode_Event__TYPE_NAME, 34, 34},
    {NULL, 0, 0},
  },
  {
    {reseq_interfaces__srv__SetMode_Request__TYPE_NAME, 36, 36},
    {NULL, 0, 0},
  },
  {
    {reseq_interfaces__srv__SetMode_Response__TYPE_NAME, 37, 37},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
reseq_interfaces__srv__SetMode__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {reseq_interfaces__srv__SetMode__TYPE_NAME, 28, 28},
      {reseq_interfaces__srv__SetMode__FIELDS, 3, 3},
    },
    {reseq_interfaces__srv__SetMode__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = reseq_interfaces__srv__SetMode_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = reseq_interfaces__srv__SetMode_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = reseq_interfaces__srv__SetMode_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char reseq_interfaces__srv__SetMode_Request__FIELD_NAME__mode[] = "mode";
static char reseq_interfaces__srv__SetMode_Request__FIELD_NAME__csv_path[] = "csv_path";

static rosidl_runtime_c__type_description__Field reseq_interfaces__srv__SetMode_Request__FIELDS[] = {
  {
    {reseq_interfaces__srv__SetMode_Request__FIELD_NAME__mode, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {reseq_interfaces__srv__SetMode_Request__FIELD_NAME__csv_path, 8, 8},
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
reseq_interfaces__srv__SetMode_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {reseq_interfaces__srv__SetMode_Request__TYPE_NAME, 36, 36},
      {reseq_interfaces__srv__SetMode_Request__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char reseq_interfaces__srv__SetMode_Response__FIELD_NAME__success[] = "success";
static char reseq_interfaces__srv__SetMode_Response__FIELD_NAME__previous_mode[] = "previous_mode";
static char reseq_interfaces__srv__SetMode_Response__FIELD_NAME__message[] = "message";

static rosidl_runtime_c__type_description__Field reseq_interfaces__srv__SetMode_Response__FIELDS[] = {
  {
    {reseq_interfaces__srv__SetMode_Response__FIELD_NAME__success, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {reseq_interfaces__srv__SetMode_Response__FIELD_NAME__previous_mode, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {reseq_interfaces__srv__SetMode_Response__FIELD_NAME__message, 7, 7},
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
reseq_interfaces__srv__SetMode_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {reseq_interfaces__srv__SetMode_Response__TYPE_NAME, 37, 37},
      {reseq_interfaces__srv__SetMode_Response__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char reseq_interfaces__srv__SetMode_Event__FIELD_NAME__info[] = "info";
static char reseq_interfaces__srv__SetMode_Event__FIELD_NAME__request[] = "request";
static char reseq_interfaces__srv__SetMode_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field reseq_interfaces__srv__SetMode_Event__FIELDS[] = {
  {
    {reseq_interfaces__srv__SetMode_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {reseq_interfaces__srv__SetMode_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {reseq_interfaces__srv__SetMode_Request__TYPE_NAME, 36, 36},
    },
    {NULL, 0, 0},
  },
  {
    {reseq_interfaces__srv__SetMode_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {reseq_interfaces__srv__SetMode_Response__TYPE_NAME, 37, 37},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription reseq_interfaces__srv__SetMode_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {reseq_interfaces__srv__SetMode_Request__TYPE_NAME, 36, 36},
    {NULL, 0, 0},
  },
  {
    {reseq_interfaces__srv__SetMode_Response__TYPE_NAME, 37, 37},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
reseq_interfaces__srv__SetMode_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {reseq_interfaces__srv__SetMode_Event__TYPE_NAME, 34, 34},
      {reseq_interfaces__srv__SetMode_Event__FIELDS, 3, 3},
    },
    {reseq_interfaces__srv__SetMode_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = reseq_interfaces__srv__SetMode_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = reseq_interfaces__srv__SetMode_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Request\n"
  "uint8 mode           # 0=OFF,1=INITIALIZING,2=SENSOR_CRATE,3=MAPPING\n"
  "string csv_path\n"
  "---\n"
  "# Response\n"
  "bool success\n"
  "uint8 previous_mode\n"
  "string message";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
reseq_interfaces__srv__SetMode__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {reseq_interfaces__srv__SetMode__TYPE_NAME, 28, 28},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 157, 157},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
reseq_interfaces__srv__SetMode_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {reseq_interfaces__srv__SetMode_Request__TYPE_NAME, 36, 36},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
reseq_interfaces__srv__SetMode_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {reseq_interfaces__srv__SetMode_Response__TYPE_NAME, 37, 37},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
reseq_interfaces__srv__SetMode_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {reseq_interfaces__srv__SetMode_Event__TYPE_NAME, 34, 34},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
reseq_interfaces__srv__SetMode__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *reseq_interfaces__srv__SetMode__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *reseq_interfaces__srv__SetMode_Event__get_individual_type_description_source(NULL);
    sources[3] = *reseq_interfaces__srv__SetMode_Request__get_individual_type_description_source(NULL);
    sources[4] = *reseq_interfaces__srv__SetMode_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
reseq_interfaces__srv__SetMode_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *reseq_interfaces__srv__SetMode_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
reseq_interfaces__srv__SetMode_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *reseq_interfaces__srv__SetMode_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
reseq_interfaces__srv__SetMode_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *reseq_interfaces__srv__SetMode_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *reseq_interfaces__srv__SetMode_Request__get_individual_type_description_source(NULL);
    sources[3] = *reseq_interfaces__srv__SetMode_Response__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
