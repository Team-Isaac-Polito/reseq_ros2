// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from reseq_interfaces:srv/GetStatus.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "reseq_interfaces/srv/detail/get_status__functions.h"
#include "reseq_interfaces/srv/detail/get_status__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace reseq_interfaces
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _GetStatus_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetStatus_Request_type_support_ids_t;

static const _GetStatus_Request_type_support_ids_t _GetStatus_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _GetStatus_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetStatus_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetStatus_Request_type_support_symbol_names_t _GetStatus_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, reseq_interfaces, srv, GetStatus_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, reseq_interfaces, srv, GetStatus_Request)),
  }
};

typedef struct _GetStatus_Request_type_support_data_t
{
  void * data[2];
} _GetStatus_Request_type_support_data_t;

static _GetStatus_Request_type_support_data_t _GetStatus_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetStatus_Request_message_typesupport_map = {
  2,
  "reseq_interfaces",
  &_GetStatus_Request_message_typesupport_ids.typesupport_identifier[0],
  &_GetStatus_Request_message_typesupport_symbol_names.symbol_name[0],
  &_GetStatus_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t GetStatus_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetStatus_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &reseq_interfaces__srv__GetStatus_Request__get_type_hash,
  &reseq_interfaces__srv__GetStatus_Request__get_type_description,
  &reseq_interfaces__srv__GetStatus_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace reseq_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<reseq_interfaces::srv::GetStatus_Request>()
{
  return &::reseq_interfaces::srv::rosidl_typesupport_cpp::GetStatus_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, reseq_interfaces, srv, GetStatus_Request)() {
  return get_message_type_support_handle<reseq_interfaces::srv::GetStatus_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "reseq_interfaces/srv/detail/get_status__functions.h"
// already included above
// #include "reseq_interfaces/srv/detail/get_status__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace reseq_interfaces
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _GetStatus_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetStatus_Response_type_support_ids_t;

static const _GetStatus_Response_type_support_ids_t _GetStatus_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _GetStatus_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetStatus_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetStatus_Response_type_support_symbol_names_t _GetStatus_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, reseq_interfaces, srv, GetStatus_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, reseq_interfaces, srv, GetStatus_Response)),
  }
};

typedef struct _GetStatus_Response_type_support_data_t
{
  void * data[2];
} _GetStatus_Response_type_support_data_t;

static _GetStatus_Response_type_support_data_t _GetStatus_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetStatus_Response_message_typesupport_map = {
  2,
  "reseq_interfaces",
  &_GetStatus_Response_message_typesupport_ids.typesupport_identifier[0],
  &_GetStatus_Response_message_typesupport_symbol_names.symbol_name[0],
  &_GetStatus_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t GetStatus_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetStatus_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &reseq_interfaces__srv__GetStatus_Response__get_type_hash,
  &reseq_interfaces__srv__GetStatus_Response__get_type_description,
  &reseq_interfaces__srv__GetStatus_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace reseq_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<reseq_interfaces::srv::GetStatus_Response>()
{
  return &::reseq_interfaces::srv::rosidl_typesupport_cpp::GetStatus_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, reseq_interfaces, srv, GetStatus_Response)() {
  return get_message_type_support_handle<reseq_interfaces::srv::GetStatus_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "reseq_interfaces/srv/detail/get_status__functions.h"
// already included above
// #include "reseq_interfaces/srv/detail/get_status__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace reseq_interfaces
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _GetStatus_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetStatus_Event_type_support_ids_t;

static const _GetStatus_Event_type_support_ids_t _GetStatus_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _GetStatus_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetStatus_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetStatus_Event_type_support_symbol_names_t _GetStatus_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, reseq_interfaces, srv, GetStatus_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, reseq_interfaces, srv, GetStatus_Event)),
  }
};

typedef struct _GetStatus_Event_type_support_data_t
{
  void * data[2];
} _GetStatus_Event_type_support_data_t;

static _GetStatus_Event_type_support_data_t _GetStatus_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetStatus_Event_message_typesupport_map = {
  2,
  "reseq_interfaces",
  &_GetStatus_Event_message_typesupport_ids.typesupport_identifier[0],
  &_GetStatus_Event_message_typesupport_symbol_names.symbol_name[0],
  &_GetStatus_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t GetStatus_Event_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetStatus_Event_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &reseq_interfaces__srv__GetStatus_Event__get_type_hash,
  &reseq_interfaces__srv__GetStatus_Event__get_type_description,
  &reseq_interfaces__srv__GetStatus_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace reseq_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<reseq_interfaces::srv::GetStatus_Event>()
{
  return &::reseq_interfaces::srv::rosidl_typesupport_cpp::GetStatus_Event_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, reseq_interfaces, srv, GetStatus_Event)() {
  return get_message_type_support_handle<reseq_interfaces::srv::GetStatus_Event>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "reseq_interfaces/srv/detail/get_status__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace reseq_interfaces
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _GetStatus_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetStatus_type_support_ids_t;

static const _GetStatus_type_support_ids_t _GetStatus_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _GetStatus_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetStatus_type_support_symbol_names_t;
#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetStatus_type_support_symbol_names_t _GetStatus_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, reseq_interfaces, srv, GetStatus)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, reseq_interfaces, srv, GetStatus)),
  }
};

typedef struct _GetStatus_type_support_data_t
{
  void * data[2];
} _GetStatus_type_support_data_t;

static _GetStatus_type_support_data_t _GetStatus_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetStatus_service_typesupport_map = {
  2,
  "reseq_interfaces",
  &_GetStatus_service_typesupport_ids.typesupport_identifier[0],
  &_GetStatus_service_typesupport_symbol_names.symbol_name[0],
  &_GetStatus_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t GetStatus_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetStatus_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
  ::rosidl_typesupport_cpp::get_message_type_support_handle<reseq_interfaces::srv::GetStatus_Request>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<reseq_interfaces::srv::GetStatus_Response>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<reseq_interfaces::srv::GetStatus_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<reseq_interfaces::srv::GetStatus>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<reseq_interfaces::srv::GetStatus>,
  &reseq_interfaces__srv__GetStatus__get_type_hash,
  &reseq_interfaces__srv__GetStatus__get_type_description,
  &reseq_interfaces__srv__GetStatus__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace reseq_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<reseq_interfaces::srv::GetStatus>()
{
  return &::reseq_interfaces::srv::rosidl_typesupport_cpp::GetStatus_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, reseq_interfaces, srv, GetStatus)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<reseq_interfaces::srv::GetStatus>();
}

#ifdef __cplusplus
}
#endif
