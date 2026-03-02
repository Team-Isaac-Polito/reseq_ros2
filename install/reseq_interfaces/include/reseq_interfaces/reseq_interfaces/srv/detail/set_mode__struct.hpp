// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from reseq_interfaces:srv/SetMode.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "reseq_interfaces/srv/set_mode.hpp"


#ifndef RESEQ_INTERFACES__SRV__DETAIL__SET_MODE__STRUCT_HPP_
#define RESEQ_INTERFACES__SRV__DETAIL__SET_MODE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__reseq_interfaces__srv__SetMode_Request __attribute__((deprecated))
#else
# define DEPRECATED__reseq_interfaces__srv__SetMode_Request __declspec(deprecated)
#endif

namespace reseq_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetMode_Request_
{
  using Type = SetMode_Request_<ContainerAllocator>;

  explicit SetMode_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode = 0;
      this->csv_path = "";
    }
  }

  explicit SetMode_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : csv_path(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode = 0;
      this->csv_path = "";
    }
  }

  // field types and members
  using _mode_type =
    uint8_t;
  _mode_type mode;
  using _csv_path_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _csv_path_type csv_path;

  // setters for named parameter idiom
  Type & set__mode(
    const uint8_t & _arg)
  {
    this->mode = _arg;
    return *this;
  }
  Type & set__csv_path(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->csv_path = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    reseq_interfaces::srv::SetMode_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const reseq_interfaces::srv::SetMode_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<reseq_interfaces::srv::SetMode_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<reseq_interfaces::srv::SetMode_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      reseq_interfaces::srv::SetMode_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<reseq_interfaces::srv::SetMode_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      reseq_interfaces::srv::SetMode_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<reseq_interfaces::srv::SetMode_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<reseq_interfaces::srv::SetMode_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<reseq_interfaces::srv::SetMode_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__reseq_interfaces__srv__SetMode_Request
    std::shared_ptr<reseq_interfaces::srv::SetMode_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__reseq_interfaces__srv__SetMode_Request
    std::shared_ptr<reseq_interfaces::srv::SetMode_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetMode_Request_ & other) const
  {
    if (this->mode != other.mode) {
      return false;
    }
    if (this->csv_path != other.csv_path) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetMode_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetMode_Request_

// alias to use template instance with default allocator
using SetMode_Request =
  reseq_interfaces::srv::SetMode_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace reseq_interfaces


#ifndef _WIN32
# define DEPRECATED__reseq_interfaces__srv__SetMode_Response __attribute__((deprecated))
#else
# define DEPRECATED__reseq_interfaces__srv__SetMode_Response __declspec(deprecated)
#endif

namespace reseq_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetMode_Response_
{
  using Type = SetMode_Response_<ContainerAllocator>;

  explicit SetMode_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->previous_mode = 0;
      this->message = "";
    }
  }

  explicit SetMode_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->previous_mode = 0;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _previous_mode_type =
    uint8_t;
  _previous_mode_type previous_mode;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__previous_mode(
    const uint8_t & _arg)
  {
    this->previous_mode = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    reseq_interfaces::srv::SetMode_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const reseq_interfaces::srv::SetMode_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<reseq_interfaces::srv::SetMode_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<reseq_interfaces::srv::SetMode_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      reseq_interfaces::srv::SetMode_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<reseq_interfaces::srv::SetMode_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      reseq_interfaces::srv::SetMode_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<reseq_interfaces::srv::SetMode_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<reseq_interfaces::srv::SetMode_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<reseq_interfaces::srv::SetMode_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__reseq_interfaces__srv__SetMode_Response
    std::shared_ptr<reseq_interfaces::srv::SetMode_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__reseq_interfaces__srv__SetMode_Response
    std::shared_ptr<reseq_interfaces::srv::SetMode_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetMode_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->previous_mode != other.previous_mode) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetMode_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetMode_Response_

// alias to use template instance with default allocator
using SetMode_Response =
  reseq_interfaces::srv::SetMode_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace reseq_interfaces


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__reseq_interfaces__srv__SetMode_Event __attribute__((deprecated))
#else
# define DEPRECATED__reseq_interfaces__srv__SetMode_Event __declspec(deprecated)
#endif

namespace reseq_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetMode_Event_
{
  using Type = SetMode_Event_<ContainerAllocator>;

  explicit SetMode_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit SetMode_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<reseq_interfaces::srv::SetMode_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<reseq_interfaces::srv::SetMode_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<reseq_interfaces::srv::SetMode_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<reseq_interfaces::srv::SetMode_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<reseq_interfaces::srv::SetMode_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<reseq_interfaces::srv::SetMode_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<reseq_interfaces::srv::SetMode_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<reseq_interfaces::srv::SetMode_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    reseq_interfaces::srv::SetMode_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const reseq_interfaces::srv::SetMode_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<reseq_interfaces::srv::SetMode_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<reseq_interfaces::srv::SetMode_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      reseq_interfaces::srv::SetMode_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<reseq_interfaces::srv::SetMode_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      reseq_interfaces::srv::SetMode_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<reseq_interfaces::srv::SetMode_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<reseq_interfaces::srv::SetMode_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<reseq_interfaces::srv::SetMode_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__reseq_interfaces__srv__SetMode_Event
    std::shared_ptr<reseq_interfaces::srv::SetMode_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__reseq_interfaces__srv__SetMode_Event
    std::shared_ptr<reseq_interfaces::srv::SetMode_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetMode_Event_ & other) const
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
  bool operator!=(const SetMode_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetMode_Event_

// alias to use template instance with default allocator
using SetMode_Event =
  reseq_interfaces::srv::SetMode_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace reseq_interfaces

namespace reseq_interfaces
{

namespace srv
{

struct SetMode
{
  using Request = reseq_interfaces::srv::SetMode_Request;
  using Response = reseq_interfaces::srv::SetMode_Response;
  using Event = reseq_interfaces::srv::SetMode_Event;
};

}  // namespace srv

}  // namespace reseq_interfaces

#endif  // RESEQ_INTERFACES__SRV__DETAIL__SET_MODE__STRUCT_HPP_
