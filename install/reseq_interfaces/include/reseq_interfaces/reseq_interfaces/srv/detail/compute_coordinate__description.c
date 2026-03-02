// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from reseq_interfaces:srv/ComputeCoordinate.idl
// generated code does not contain a copyright notice

#include "reseq_interfaces/srv/detail/compute_coordinate__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
const rosidl_type_hash_t *
reseq_interfaces__srv__ComputeCoordinate__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xf3, 0x27, 0x8d, 0xa9, 0xf2, 0xa7, 0xba, 0x30,
      0xd6, 0x59, 0x5d, 0x3b, 0x62, 0xdf, 0x4f, 0xd0,
      0xc6, 0x55, 0x80, 0xe0, 0x6f, 0x44, 0x36, 0x7b,
      0xf2, 0x0d, 0xdd, 0x35, 0xf6, 0x57, 0x5b, 0xec,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
const rosidl_type_hash_t *
reseq_interfaces__srv__ComputeCoordinate_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xff, 0x25, 0x46, 0x3b, 0x4f, 0x4d, 0x27, 0x95,
      0x85, 0xd2, 0x79, 0x1d, 0x48, 0x36, 0x7d, 0x81,
      0xb4, 0xb6, 0x16, 0xe2, 0x33, 0x98, 0x38, 0x6b,
      0x2b, 0x34, 0xf6, 0xd1, 0xca, 0xeb, 0xc2, 0x24,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
const rosidl_type_hash_t *
reseq_interfaces__srv__ComputeCoordinate_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xc2, 0xbb, 0x7f, 0xc4, 0x03, 0x3f, 0xf6, 0xb1,
      0xb4, 0xca, 0xc2, 0xfc, 0xeb, 0x9a, 0xae, 0xff,
      0xff, 0x50, 0xfb, 0xb7, 0x7d, 0xa1, 0x5f, 0x05,
      0x2b, 0x29, 0x5b, 0x8b, 0x5b, 0xc9, 0xbf, 0xd9,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
const rosidl_type_hash_t *
reseq_interfaces__srv__ComputeCoordinate_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x6f, 0x46, 0x8b, 0xf3, 0x9e, 0x95, 0x13, 0xd0,
      0x77, 0x54, 0x85, 0xac, 0x7a, 0xc7, 0x84, 0x7d,
      0x66, 0x2b, 0x63, 0xf2, 0x25, 0x9f, 0x63, 0xcf,
      0x49, 0x9c, 0x0f, 0xb2, 0xbf, 0x6b, 0x91, 0x0e,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "sensor_msgs/msg/detail/region_of_interest__functions.h"
#include "reseq_interfaces/msg/detail/detection__functions.h"
#include "geometry_msgs/msg/detail/point__functions.h"
#include "std_msgs/msg/detail/header__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "service_msgs/msg/detail/service_event_info__functions.h"
#include "geometry_msgs/msg/detail/point_stamped__functions.h"
#include "sensor_msgs/msg/detail/camera_info__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__Point__EXPECTED_HASH = {1, {
    0x69, 0x63, 0x08, 0x48, 0x42, 0xa9, 0xb0, 0x44,
    0x94, 0xd6, 0xb2, 0x94, 0x1d, 0x11, 0x44, 0x47,
    0x08, 0xd8, 0x92, 0xda, 0x2f, 0x4b, 0x09, 0x84,
    0x3b, 0x9c, 0x43, 0xf4, 0x2a, 0x7f, 0x68, 0x81,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__PointStamped__EXPECTED_HASH = {1, {
    0x4c, 0x02, 0x96, 0xaf, 0x86, 0xe0, 0x1e, 0x56,
    0x2e, 0x9e, 0x04, 0x05, 0xd1, 0x38, 0xa0, 0x15,
    0x37, 0x24, 0x75, 0x80, 0x07, 0x6c, 0x58, 0xea,
    0x38, 0xd7, 0x92, 0x3a, 0xc1, 0x04, 0x58, 0x97,
  }};
static const rosidl_type_hash_t reseq_interfaces__msg__Detection__EXPECTED_HASH = {1, {
    0x48, 0xbd, 0xf2, 0x56, 0xa8, 0x5e, 0x9b, 0xb4,
    0x42, 0xa2, 0x0d, 0xb9, 0x60, 0xcf, 0xd0, 0xaa,
    0x75, 0x15, 0x7f, 0xbc, 0xce, 0x0a, 0xc4, 0x67,
    0x97, 0x07, 0x00, 0xd7, 0xd7, 0xd9, 0x75, 0x5d,
  }};
static const rosidl_type_hash_t sensor_msgs__msg__CameraInfo__EXPECTED_HASH = {1, {
    0xb3, 0xdf, 0xd6, 0x8f, 0xf4, 0x6c, 0x9d, 0x56,
    0xc8, 0x0f, 0xd3, 0xbd, 0x4e, 0xd2, 0x2c, 0x7a,
    0x4d, 0xdc, 0xe8, 0xc8, 0x34, 0x8f, 0x2f, 0x59,
    0xc2, 0x99, 0xe7, 0x31, 0x18, 0xe7, 0xe2, 0x75,
  }};
static const rosidl_type_hash_t sensor_msgs__msg__RegionOfInterest__EXPECTED_HASH = {1, {
    0xad, 0x16, 0xbc, 0xba, 0x5f, 0x91, 0x31, 0xdc,
    0xdb, 0xa6, 0xfb, 0xde, 0xd1, 0x9f, 0x72, 0x6f,
    0x54, 0x40, 0xe3, 0xc5, 0x13, 0xb4, 0xfb, 0x58,
    0x6d, 0xd3, 0x02, 0x7e, 0xee, 0xd8, 0xab, 0xb1,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char reseq_interfaces__srv__ComputeCoordinate__TYPE_NAME[] = "reseq_interfaces/srv/ComputeCoordinate";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char geometry_msgs__msg__Point__TYPE_NAME[] = "geometry_msgs/msg/Point";
static char geometry_msgs__msg__PointStamped__TYPE_NAME[] = "geometry_msgs/msg/PointStamped";
static char reseq_interfaces__msg__Detection__TYPE_NAME[] = "reseq_interfaces/msg/Detection";
static char reseq_interfaces__srv__ComputeCoordinate_Event__TYPE_NAME[] = "reseq_interfaces/srv/ComputeCoordinate_Event";
static char reseq_interfaces__srv__ComputeCoordinate_Request__TYPE_NAME[] = "reseq_interfaces/srv/ComputeCoordinate_Request";
static char reseq_interfaces__srv__ComputeCoordinate_Response__TYPE_NAME[] = "reseq_interfaces/srv/ComputeCoordinate_Response";
static char sensor_msgs__msg__CameraInfo__TYPE_NAME[] = "sensor_msgs/msg/CameraInfo";
static char sensor_msgs__msg__RegionOfInterest__TYPE_NAME[] = "sensor_msgs/msg/RegionOfInterest";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char reseq_interfaces__srv__ComputeCoordinate__FIELD_NAME__request_message[] = "request_message";
static char reseq_interfaces__srv__ComputeCoordinate__FIELD_NAME__response_message[] = "response_message";
static char reseq_interfaces__srv__ComputeCoordinate__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field reseq_interfaces__srv__ComputeCoordinate__FIELDS[] = {
  {
    {reseq_interfaces__srv__ComputeCoordinate__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {reseq_interfaces__srv__ComputeCoordinate_Request__TYPE_NAME, 46, 46},
    },
    {NULL, 0, 0},
  },
  {
    {reseq_interfaces__srv__ComputeCoordinate__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {reseq_interfaces__srv__ComputeCoordinate_Response__TYPE_NAME, 47, 47},
    },
    {NULL, 0, 0},
  },
  {
    {reseq_interfaces__srv__ComputeCoordinate__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {reseq_interfaces__srv__ComputeCoordinate_Event__TYPE_NAME, 44, 44},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription reseq_interfaces__srv__ComputeCoordinate__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Point__TYPE_NAME, 23, 23},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__PointStamped__TYPE_NAME, 30, 30},
    {NULL, 0, 0},
  },
  {
    {reseq_interfaces__msg__Detection__TYPE_NAME, 30, 30},
    {NULL, 0, 0},
  },
  {
    {reseq_interfaces__srv__ComputeCoordinate_Event__TYPE_NAME, 44, 44},
    {NULL, 0, 0},
  },
  {
    {reseq_interfaces__srv__ComputeCoordinate_Request__TYPE_NAME, 46, 46},
    {NULL, 0, 0},
  },
  {
    {reseq_interfaces__srv__ComputeCoordinate_Response__TYPE_NAME, 47, 47},
    {NULL, 0, 0},
  },
  {
    {sensor_msgs__msg__CameraInfo__TYPE_NAME, 26, 26},
    {NULL, 0, 0},
  },
  {
    {sensor_msgs__msg__RegionOfInterest__TYPE_NAME, 32, 32},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
reseq_interfaces__srv__ComputeCoordinate__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {reseq_interfaces__srv__ComputeCoordinate__TYPE_NAME, 38, 38},
      {reseq_interfaces__srv__ComputeCoordinate__FIELDS, 3, 3},
    },
    {reseq_interfaces__srv__ComputeCoordinate__REFERENCED_TYPE_DESCRIPTIONS, 11, 11},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Point__EXPECTED_HASH, geometry_msgs__msg__Point__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = geometry_msgs__msg__Point__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__PointStamped__EXPECTED_HASH, geometry_msgs__msg__PointStamped__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = geometry_msgs__msg__PointStamped__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&reseq_interfaces__msg__Detection__EXPECTED_HASH, reseq_interfaces__msg__Detection__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = reseq_interfaces__msg__Detection__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[4].fields = reseq_interfaces__srv__ComputeCoordinate_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[5].fields = reseq_interfaces__srv__ComputeCoordinate_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[6].fields = reseq_interfaces__srv__ComputeCoordinate_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&sensor_msgs__msg__CameraInfo__EXPECTED_HASH, sensor_msgs__msg__CameraInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[7].fields = sensor_msgs__msg__CameraInfo__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&sensor_msgs__msg__RegionOfInterest__EXPECTED_HASH, sensor_msgs__msg__RegionOfInterest__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[8].fields = sensor_msgs__msg__RegionOfInterest__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[9].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[10].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char reseq_interfaces__srv__ComputeCoordinate_Request__FIELD_NAME__detection[] = "detection";
static char reseq_interfaces__srv__ComputeCoordinate_Request__FIELD_NAME__camera_info[] = "camera_info";
static char reseq_interfaces__srv__ComputeCoordinate_Request__FIELD_NAME__target_frame[] = "target_frame";

static rosidl_runtime_c__type_description__Field reseq_interfaces__srv__ComputeCoordinate_Request__FIELDS[] = {
  {
    {reseq_interfaces__srv__ComputeCoordinate_Request__FIELD_NAME__detection, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {reseq_interfaces__msg__Detection__TYPE_NAME, 30, 30},
    },
    {NULL, 0, 0},
  },
  {
    {reseq_interfaces__srv__ComputeCoordinate_Request__FIELD_NAME__camera_info, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {sensor_msgs__msg__CameraInfo__TYPE_NAME, 26, 26},
    },
    {NULL, 0, 0},
  },
  {
    {reseq_interfaces__srv__ComputeCoordinate_Request__FIELD_NAME__target_frame, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription reseq_interfaces__srv__ComputeCoordinate_Request__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {reseq_interfaces__msg__Detection__TYPE_NAME, 30, 30},
    {NULL, 0, 0},
  },
  {
    {sensor_msgs__msg__CameraInfo__TYPE_NAME, 26, 26},
    {NULL, 0, 0},
  },
  {
    {sensor_msgs__msg__RegionOfInterest__TYPE_NAME, 32, 32},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
reseq_interfaces__srv__ComputeCoordinate_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {reseq_interfaces__srv__ComputeCoordinate_Request__TYPE_NAME, 46, 46},
      {reseq_interfaces__srv__ComputeCoordinate_Request__FIELDS, 3, 3},
    },
    {reseq_interfaces__srv__ComputeCoordinate_Request__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&reseq_interfaces__msg__Detection__EXPECTED_HASH, reseq_interfaces__msg__Detection__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = reseq_interfaces__msg__Detection__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&sensor_msgs__msg__CameraInfo__EXPECTED_HASH, sensor_msgs__msg__CameraInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = sensor_msgs__msg__CameraInfo__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&sensor_msgs__msg__RegionOfInterest__EXPECTED_HASH, sensor_msgs__msg__RegionOfInterest__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = sensor_msgs__msg__RegionOfInterest__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char reseq_interfaces__srv__ComputeCoordinate_Response__FIELD_NAME__success[] = "success";
static char reseq_interfaces__srv__ComputeCoordinate_Response__FIELD_NAME__point[] = "point";
static char reseq_interfaces__srv__ComputeCoordinate_Response__FIELD_NAME__message[] = "message";

static rosidl_runtime_c__type_description__Field reseq_interfaces__srv__ComputeCoordinate_Response__FIELDS[] = {
  {
    {reseq_interfaces__srv__ComputeCoordinate_Response__FIELD_NAME__success, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {reseq_interfaces__srv__ComputeCoordinate_Response__FIELD_NAME__point, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__PointStamped__TYPE_NAME, 30, 30},
    },
    {NULL, 0, 0},
  },
  {
    {reseq_interfaces__srv__ComputeCoordinate_Response__FIELD_NAME__message, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription reseq_interfaces__srv__ComputeCoordinate_Response__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Point__TYPE_NAME, 23, 23},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__PointStamped__TYPE_NAME, 30, 30},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
reseq_interfaces__srv__ComputeCoordinate_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {reseq_interfaces__srv__ComputeCoordinate_Response__TYPE_NAME, 47, 47},
      {reseq_interfaces__srv__ComputeCoordinate_Response__FIELDS, 3, 3},
    },
    {reseq_interfaces__srv__ComputeCoordinate_Response__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Point__EXPECTED_HASH, geometry_msgs__msg__Point__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = geometry_msgs__msg__Point__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__PointStamped__EXPECTED_HASH, geometry_msgs__msg__PointStamped__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = geometry_msgs__msg__PointStamped__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char reseq_interfaces__srv__ComputeCoordinate_Event__FIELD_NAME__info[] = "info";
static char reseq_interfaces__srv__ComputeCoordinate_Event__FIELD_NAME__request[] = "request";
static char reseq_interfaces__srv__ComputeCoordinate_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field reseq_interfaces__srv__ComputeCoordinate_Event__FIELDS[] = {
  {
    {reseq_interfaces__srv__ComputeCoordinate_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {reseq_interfaces__srv__ComputeCoordinate_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {reseq_interfaces__srv__ComputeCoordinate_Request__TYPE_NAME, 46, 46},
    },
    {NULL, 0, 0},
  },
  {
    {reseq_interfaces__srv__ComputeCoordinate_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {reseq_interfaces__srv__ComputeCoordinate_Response__TYPE_NAME, 47, 47},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription reseq_interfaces__srv__ComputeCoordinate_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Point__TYPE_NAME, 23, 23},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__PointStamped__TYPE_NAME, 30, 30},
    {NULL, 0, 0},
  },
  {
    {reseq_interfaces__msg__Detection__TYPE_NAME, 30, 30},
    {NULL, 0, 0},
  },
  {
    {reseq_interfaces__srv__ComputeCoordinate_Request__TYPE_NAME, 46, 46},
    {NULL, 0, 0},
  },
  {
    {reseq_interfaces__srv__ComputeCoordinate_Response__TYPE_NAME, 47, 47},
    {NULL, 0, 0},
  },
  {
    {sensor_msgs__msg__CameraInfo__TYPE_NAME, 26, 26},
    {NULL, 0, 0},
  },
  {
    {sensor_msgs__msg__RegionOfInterest__TYPE_NAME, 32, 32},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
reseq_interfaces__srv__ComputeCoordinate_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {reseq_interfaces__srv__ComputeCoordinate_Event__TYPE_NAME, 44, 44},
      {reseq_interfaces__srv__ComputeCoordinate_Event__FIELDS, 3, 3},
    },
    {reseq_interfaces__srv__ComputeCoordinate_Event__REFERENCED_TYPE_DESCRIPTIONS, 10, 10},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Point__EXPECTED_HASH, geometry_msgs__msg__Point__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = geometry_msgs__msg__Point__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__PointStamped__EXPECTED_HASH, geometry_msgs__msg__PointStamped__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = geometry_msgs__msg__PointStamped__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&reseq_interfaces__msg__Detection__EXPECTED_HASH, reseq_interfaces__msg__Detection__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = reseq_interfaces__msg__Detection__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[4].fields = reseq_interfaces__srv__ComputeCoordinate_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[5].fields = reseq_interfaces__srv__ComputeCoordinate_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&sensor_msgs__msg__CameraInfo__EXPECTED_HASH, sensor_msgs__msg__CameraInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[6].fields = sensor_msgs__msg__CameraInfo__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&sensor_msgs__msg__RegionOfInterest__EXPECTED_HASH, sensor_msgs__msg__RegionOfInterest__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[7].fields = sensor_msgs__msg__RegionOfInterest__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[8].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[9].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Request\n"
  "reseq_interfaces/Detection detection\n"
  "sensor_msgs/CameraInfo camera_info\n"
  "string target_frame\n"
  "---\n"
  "# Response\n"
  "bool success\n"
  "geometry_msgs/PointStamped point\n"
  "string message";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
reseq_interfaces__srv__ComputeCoordinate__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {reseq_interfaces__srv__ComputeCoordinate__TYPE_NAME, 38, 38},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 177, 177},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
reseq_interfaces__srv__ComputeCoordinate_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {reseq_interfaces__srv__ComputeCoordinate_Request__TYPE_NAME, 46, 46},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
reseq_interfaces__srv__ComputeCoordinate_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {reseq_interfaces__srv__ComputeCoordinate_Response__TYPE_NAME, 47, 47},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
reseq_interfaces__srv__ComputeCoordinate_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {reseq_interfaces__srv__ComputeCoordinate_Event__TYPE_NAME, 44, 44},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
reseq_interfaces__srv__ComputeCoordinate__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[12];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 12, 12};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *reseq_interfaces__srv__ComputeCoordinate__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *geometry_msgs__msg__Point__get_individual_type_description_source(NULL);
    sources[3] = *geometry_msgs__msg__PointStamped__get_individual_type_description_source(NULL);
    sources[4] = *reseq_interfaces__msg__Detection__get_individual_type_description_source(NULL);
    sources[5] = *reseq_interfaces__srv__ComputeCoordinate_Event__get_individual_type_description_source(NULL);
    sources[6] = *reseq_interfaces__srv__ComputeCoordinate_Request__get_individual_type_description_source(NULL);
    sources[7] = *reseq_interfaces__srv__ComputeCoordinate_Response__get_individual_type_description_source(NULL);
    sources[8] = *sensor_msgs__msg__CameraInfo__get_individual_type_description_source(NULL);
    sources[9] = *sensor_msgs__msg__RegionOfInterest__get_individual_type_description_source(NULL);
    sources[10] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[11] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
reseq_interfaces__srv__ComputeCoordinate_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *reseq_interfaces__srv__ComputeCoordinate_Request__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *reseq_interfaces__msg__Detection__get_individual_type_description_source(NULL);
    sources[3] = *sensor_msgs__msg__CameraInfo__get_individual_type_description_source(NULL);
    sources[4] = *sensor_msgs__msg__RegionOfInterest__get_individual_type_description_source(NULL);
    sources[5] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
reseq_interfaces__srv__ComputeCoordinate_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *reseq_interfaces__srv__ComputeCoordinate_Response__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *geometry_msgs__msg__Point__get_individual_type_description_source(NULL);
    sources[3] = *geometry_msgs__msg__PointStamped__get_individual_type_description_source(NULL);
    sources[4] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
reseq_interfaces__srv__ComputeCoordinate_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[11];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 11, 11};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *reseq_interfaces__srv__ComputeCoordinate_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *geometry_msgs__msg__Point__get_individual_type_description_source(NULL);
    sources[3] = *geometry_msgs__msg__PointStamped__get_individual_type_description_source(NULL);
    sources[4] = *reseq_interfaces__msg__Detection__get_individual_type_description_source(NULL);
    sources[5] = *reseq_interfaces__srv__ComputeCoordinate_Request__get_individual_type_description_source(NULL);
    sources[6] = *reseq_interfaces__srv__ComputeCoordinate_Response__get_individual_type_description_source(NULL);
    sources[7] = *sensor_msgs__msg__CameraInfo__get_individual_type_description_source(NULL);
    sources[8] = *sensor_msgs__msg__RegionOfInterest__get_individual_type_description_source(NULL);
    sources[9] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[10] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
