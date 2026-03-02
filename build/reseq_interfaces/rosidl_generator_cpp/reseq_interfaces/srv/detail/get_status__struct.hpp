// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from reseq_interfaces:srv/GetStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "reseq_interfaces/srv/get_status.hpp"


#ifndef RESEQ_INTERFACES__SRV__DETAIL__GET_STATUS__STRUCT_HPP_
#define RESEQ_INTERFACES__SRV__DETAIL__GET_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__reseq_interfaces__srv__GetStatus_Request __attribute__((deprecated))
#else
# define DEPRECATED__reseq_interfaces__srv__GetStatus_Request __declspec(deprecated)
#endif

namespace reseq_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetStatus_Request_
{
  using Type = GetStatus_Request_<ContainerAllocator>;

  explicit GetStatus_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit GetStatus_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    reseq_interfaces::srv::GetStatus_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const reseq_interfaces::srv::GetStatus_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<reseq_interfaces::srv::GetStatus_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<reseq_interfaces::srv::GetStatus_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      reseq_interfaces::srv::GetStatus_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<reseq_interfaces::srv::GetStatus_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      reseq_interfaces::srv::GetStatus_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<reseq_interfaces::srv::GetStatus_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<reseq_interfaces::srv::GetStatus_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<reseq_interfaces::srv::GetStatus_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__reseq_interfaces__srv__GetStatus_Request
    std::shared_ptr<reseq_interfaces::srv::GetStatus_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__reseq_interfaces__srv__GetStatus_Request
    std::shared_ptr<reseq_interfaces::srv::GetStatus_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetStatus_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetStatus_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetStatus_Request_

// alias to use template instance with default allocator
using GetStatus_Request =
  reseq_interfaces::srv::GetStatus_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace reseq_interfaces


#ifndef _WIN32
# define DEPRECATED__reseq_interfaces__srv__GetStatus_Response __attribute__((deprecated))
#else
# define DEPRECATED__reseq_interfaces__srv__GetStatus_Response __declspec(deprecated)
#endif

namespace reseq_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetStatus_Response_
{
  using Type = GetStatus_Response_<ContainerAllocator>;

  explicit GetStatus_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->current_mode = 0;
      this->initialized = false;
      this->csv_path = "";
      this->last_error = "";
    }
  }

  explicit GetStatus_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : csv_path(_alloc),
    last_error(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->current_mode = 0;
      this->initialized = false;
      this->csv_path = "";
      this->last_error = "";
    }
  }

  // field types and members
  using _current_mode_type =
    uint8_t;
  _current_mode_type current_mode;
  using _initialized_type =
    bool;
  _initialized_type initialized;
  using _csv_path_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _csv_path_type csv_path;
  using _last_error_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _last_error_type last_error;

  // setters for named parameter idiom
  Type & set__current_mode(
    const uint8_t & _arg)
  {
    this->current_mode = _arg;
    return *this;
  }
  Type & set__initialized(
    const bool & _arg)
  {
    this->initialized = _arg;
    return *this;
  }
  Type & set__csv_path(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->csv_path = _arg;
    return *this;
  }
  Type & set__last_error(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->last_error = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    reseq_interfaces::srv::GetStatus_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const reseq_interfaces::srv::GetStatus_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<reseq_interfaces::srv::GetStatus_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<reseq_interfaces::srv::GetStatus_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      reseq_interfaces::srv::GetStatus_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<reseq_interfaces::srv::GetStatus_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      reseq_interfaces::srv::GetStatus_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<reseq_interfaces::srv::GetStatus_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<reseq_interfaces::srv::GetStatus_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<reseq_interfaces::srv::GetStatus_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__reseq_interfaces__srv__GetStatus_Response
    std::shared_ptr<reseq_interfaces::srv::GetStatus_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__reseq_interfaces__srv__GetStatus_Response
    std::shared_ptr<reseq_interfaces::srv::GetStatus_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetStatus_Response_ & other) const
  {
    if (this->current_mode != other.current_mode) {
      return false;
    }
    if (this->initialized != other.initialized) {
      return false;
    }
    if (this->csv_path != other.csv_path) {
      return false;
    }
    if (this->last_error != other.last_error) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetStatus_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetStatus_Response_

// alias to use template instance with default allocator
using GetStatus_Response =
  reseq_interfaces::srv::GetStatus_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace reseq_interfaces


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__reseq_interfaces__srv__GetStatus_Event __attribute__((deprecated))
#else
# define DEPRECATED__reseq_interfaces__srv__GetStatus_Event __declspec(deprecated)
#endif

namespace reseq_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetStatus_Event_
{
  using Type = GetStatus_Event_<ContainerAllocator>;

  explicit GetStatus_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit GetStatus_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<reseq_interfaces::srv::GetStatus_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<reseq_interfaces::srv::GetStatus_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<reseq_interfaces::srv::GetStatus_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<reseq_interfaces::srv::GetStatus_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<reseq_interfaces::srv::GetStatus_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<reseq_interfaces::srv::GetStatus_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<reseq_interfaces::srv::GetStatus_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<reseq_interfaces::srv::GetStatus_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    reseq_interfaces::srv::GetStatus_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const reseq_interfaces::srv::GetStatus_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<reseq_interfaces::srv::GetStatus_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<reseq_interfaces::srv::GetStatus_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      reseq_interfaces::srv::GetStatus_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<reseq_interfaces::srv::GetStatus_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      reseq_interfaces::srv::GetStatus_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<reseq_interfaces::srv::GetStatus_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<reseq_interfaces::srv::GetStatus_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<reseq_interfaces::srv::GetStatus_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__reseq_interfaces__srv__GetStatus_Event
    std::shared_ptr<reseq_interfaces::srv::GetStatus_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__reseq_interfaces__srv__GetStatus_Event
    std::shared_ptr<reseq_interfaces::srv::GetStatus_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetStatus_Event_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetStatus_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetStatus_Event_

// alias to use template instance with default allocator
using GetStatus_Event =
  reseq_interfaces::srv::GetStatus_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace reseq_interfaces

namespace reseq_interfaces
{

namespace srv
{

struct GetStatus
{
  using Request = reseq_interfaces::srv::GetStatus_Request;
  using Response = reseq_interfaces::srv::GetStatus_Response;
  using Event = reseq_interfaces::srv::GetStatus_Event;
};

}  // namespace srv

}  // namespace reseq_interfaces

#endif  // RESEQ_INTERFACES__SRV__DETAIL__GET_STATUS__STRUCT_HPP_
