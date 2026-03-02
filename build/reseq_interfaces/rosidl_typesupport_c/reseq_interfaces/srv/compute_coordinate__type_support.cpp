// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from reseq_interfaces:srv/ComputeCoordinate.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "reseq_interfaces/srv/detail/compute_coordinate__struct.h"
#include "reseq_interfaces/srv/detail/compute_coordinate__type_support.h"
#include "reseq_interfaces/srv/detail/compute_coordinate__functions.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace reseq_interfaces
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _ComputeCoordinate_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ComputeCoordinate_Request_type_support_ids_t;

static const _ComputeCoordinate_Request_type_support_ids_t _ComputeCoordinate_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _ComputeCoordinate_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ComputeCoordinate_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ComputeCoordinate_Request_type_support_symbol_names_t _ComputeCoordinate_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, reseq_interfaces, srv, ComputeCoordinate_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, reseq_interfaces, srv, ComputeCoordinate_Request)),
  }
};

typedef struct _ComputeCoordinate_Request_type_support_data_t
{
  void * data[2];
} _ComputeCoordinate_Request_type_support_data_t;

static _ComputeCoordinate_Request_type_support_data_t _ComputeCoordinate_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ComputeCoordinate_Request_message_typesupport_map = {
  2,
  "reseq_interfaces",
  &_ComputeCoordinate_Request_message_typesupport_ids.typesupport_identifier[0],
  &_ComputeCoordinate_Request_message_typesupport_symbol_names.symbol_name[0],
  &_ComputeCoordinate_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ComputeCoordinate_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ComputeCoordinate_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &reseq_interfaces__srv__ComputeCoordinate_Request__get_type_hash,
  &reseq_interfaces__srv__ComputeCoordinate_Request__get_type_description,
  &reseq_interfaces__srv__ComputeCoordinate_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace reseq_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, reseq_interfaces, srv, ComputeCoordinate_Request)() {
  return &::reseq_interfaces::srv::rosidl_typesupport_c::ComputeCoordinate_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "reseq_interfaces/srv/detail/compute_coordinate__struct.h"
// already included above
// #include "reseq_interfaces/srv/detail/compute_coordinate__type_support.h"
// already included above
// #include "reseq_interfaces/srv/detail/compute_coordinate__functions.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace reseq_interfaces
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _ComputeCoordinate_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ComputeCoordinate_Response_type_support_ids_t;

static const _ComputeCoordinate_Response_type_support_ids_t _ComputeCoordinate_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _ComputeCoordinate_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ComputeCoordinate_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ComputeCoordinate_Response_type_support_symbol_names_t _ComputeCoordinate_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, reseq_interfaces, srv, ComputeCoordinate_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, reseq_interfaces, srv, ComputeCoordinate_Response)),
  }
};

typedef struct _ComputeCoordinate_Response_type_support_data_t
{
  void * data[2];
} _ComputeCoordinate_Response_type_support_data_t;

static _ComputeCoordinate_Response_type_support_data_t _ComputeCoordinate_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ComputeCoordinate_Response_message_typesupport_map = {
  2,
  "reseq_interfaces",
  &_ComputeCoordinate_Response_message_typesupport_ids.typesupport_identifier[0],
  &_ComputeCoordinate_Response_message_typesupport_symbol_names.symbol_name[0],
  &_ComputeCoordinate_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ComputeCoordinate_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ComputeCoordinate_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &reseq_interfaces__srv__ComputeCoordinate_Response__get_type_hash,
  &reseq_interfaces__srv__ComputeCoordinate_Response__get_type_description,
  &reseq_interfaces__srv__ComputeCoordinate_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace reseq_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, reseq_interfaces, srv, ComputeCoordinate_Response)() {
  return &::reseq_interfaces::srv::rosidl_typesupport_c::ComputeCoordinate_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "reseq_interfaces/srv/detail/compute_coordinate__struct.h"
// already included above
// #include "reseq_interfaces/srv/detail/compute_coordinate__type_support.h"
// already included above
// #include "reseq_interfaces/srv/detail/compute_coordinate__functions.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace reseq_interfaces
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _ComputeCoordinate_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ComputeCoordinate_Event_type_support_ids_t;

static const _ComputeCoordinate_Event_type_support_ids_t _ComputeCoordinate_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _ComputeCoordinate_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ComputeCoordinate_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ComputeCoordinate_Event_type_support_symbol_names_t _ComputeCoordinate_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, reseq_interfaces, srv, ComputeCoordinate_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, reseq_interfaces, srv, ComputeCoordinate_Event)),
  }
};

typedef struct _ComputeCoordinate_Event_type_support_data_t
{
  void * data[2];
} _ComputeCoordinate_Event_type_support_data_t;

static _ComputeCoordinate_Event_type_support_data_t _ComputeCoordinate_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ComputeCoordinate_Event_message_typesupport_map = {
  2,
  "reseq_interfaces",
  &_ComputeCoordinate_Event_message_typesupport_ids.typesupport_identifier[0],
  &_ComputeCoordinate_Event_message_typesupport_symbol_names.symbol_name[0],
  &_ComputeCoordinate_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ComputeCoordinate_Event_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ComputeCoordinate_Event_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &reseq_interfaces__srv__ComputeCoordinate_Event__get_type_hash,
  &reseq_interfaces__srv__ComputeCoordinate_Event__get_type_description,
  &reseq_interfaces__srv__ComputeCoordinate_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace reseq_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, reseq_interfaces, srv, ComputeCoordinate_Event)() {
  return &::reseq_interfaces::srv::rosidl_typesupport_c::ComputeCoordinate_Event_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "reseq_interfaces/srv/detail/compute_coordinate__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
#include "service_msgs/msg/service_event_info.h"
#include "builtin_interfaces/msg/time.h"

namespace reseq_interfaces
{

namespace srv
{

namespace rosidl_typesupport_c
{
typedef struct _ComputeCoordinate_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ComputeCoordinate_type_support_ids_t;

static const _ComputeCoordinate_type_support_ids_t _ComputeCoordinate_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _ComputeCoordinate_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ComputeCoordinate_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ComputeCoordinate_type_support_symbol_names_t _ComputeCoordinate_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, reseq_interfaces, srv, ComputeCoordinate)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, reseq_interfaces, srv, ComputeCoordinate)),
  }
};

typedef struct _ComputeCoordinate_type_support_data_t
{
  void * data[2];
} _ComputeCoordinate_type_support_data_t;

static _ComputeCoordinate_type_support_data_t _ComputeCoordinate_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ComputeCoordinate_service_typesupport_map = {
  2,
  "reseq_interfaces",
  &_ComputeCoordinate_service_typesupport_ids.typesupport_identifier[0],
  &_ComputeCoordinate_service_typesupport_symbol_names.symbol_name[0],
  &_ComputeCoordinate_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t ComputeCoordinate_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ComputeCoordinate_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
  &ComputeCoordinate_Request_message_type_support_handle,
  &ComputeCoordinate_Response_message_type_support_handle,
  &ComputeCoordinate_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    reseq_interfaces,
    srv,
    ComputeCoordinate
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    reseq_interfaces,
    srv,
    ComputeCoordinate
  ),
  &reseq_interfaces__srv__ComputeCoordinate__get_type_hash,
  &reseq_interfaces__srv__ComputeCoordinate__get_type_description,
  &reseq_interfaces__srv__ComputeCoordinate__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace reseq_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, reseq_interfaces, srv, ComputeCoordinate)() {
  return &::reseq_interfaces::srv::rosidl_typesupport_c::ComputeCoordinate_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif
