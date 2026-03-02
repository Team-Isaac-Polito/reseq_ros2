// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from reseq_interfaces:srv/BatchDetections2D.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "reseq_interfaces/srv/batch_detections2_d.hpp"


#ifndef RESEQ_INTERFACES__SRV__DETAIL__BATCH_DETECTIONS2_D__STRUCT_HPP_
#define RESEQ_INTERFACES__SRV__DETAIL__BATCH_DETECTIONS2_D__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'detections'
#include "vision_msgs/msg/detail/detection2_d_array__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__reseq_interfaces__srv__BatchDetections2D_Request __attribute__((deprecated))
#else
# define DEPRECATED__reseq_interfaces__srv__BatchDetections2D_Request __declspec(deprecated)
#endif

namespace reseq_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct BatchDetections2D_Request_
{
  using Type = BatchDetections2D_Request_<ContainerAllocator>;

  explicit BatchDetections2D_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : detections(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot = "";
      this->mode = "";
    }
  }

  explicit BatchDetections2D_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : robot(_alloc),
    mode(_alloc),
    detections(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot = "";
      this->mode = "";
    }
  }

  // field types and members
  using _robot_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _robot_type robot;
  using _mode_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _mode_type mode;
  using _detections_type =
    vision_msgs::msg::Detection2DArray_<ContainerAllocator>;
  _detections_type detections;

  // setters for named parameter idiom
  Type & set__robot(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->robot = _arg;
    return *this;
  }
  Type & set__mode(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->mode = _arg;
    return *this;
  }
  Type & set__detections(
    const vision_msgs::msg::Detection2DArray_<ContainerAllocator> & _arg)
  {
    this->detections = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    reseq_interfaces::srv::BatchDetections2D_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const reseq_interfaces::srv::BatchDetections2D_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<reseq_interfaces::srv::BatchDetections2D_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<reseq_interfaces::srv::BatchDetections2D_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      reseq_interfaces::srv::BatchDetections2D_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<reseq_interfaces::srv::BatchDetections2D_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      reseq_interfaces::srv::BatchDetections2D_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<reseq_interfaces::srv::BatchDetections2D_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<reseq_interfaces::srv::BatchDetections2D_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<reseq_interfaces::srv::BatchDetections2D_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__reseq_interfaces__srv__BatchDetections2D_Request
    std::shared_ptr<reseq_interfaces::srv::BatchDetections2D_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__reseq_interfaces__srv__BatchDetections2D_Request
    std::shared_ptr<reseq_interfaces::srv::BatchDetections2D_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BatchDetections2D_Request_ & other) const
  {
    if (this->robot != other.robot) {
      return false;
    }
    if (this->mode != other.mode) {
      return false;
    }
    if (this->detections != other.detections) {
      return false;
    }
    return true;
  }
  bool operator!=(const BatchDetections2D_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BatchDetections2D_Request_

// alias to use template instance with default allocator
using BatchDetections2D_Request =
  reseq_interfaces::srv::BatchDetections2D_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace reseq_interfaces


#ifndef _WIN32
# define DEPRECATED__reseq_interfaces__srv__BatchDetections2D_Response __attribute__((deprecated))
#else
# define DEPRECATED__reseq_interfaces__srv__BatchDetections2D_Response __declspec(deprecated)
#endif

namespace reseq_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct BatchDetections2D_Response_
{
  using Type = BatchDetections2D_Response_<ContainerAllocator>;

  explicit BatchDetections2D_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit BatchDetections2D_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
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
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    reseq_interfaces::srv::BatchDetections2D_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const reseq_interfaces::srv::BatchDetections2D_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<reseq_interfaces::srv::BatchDetections2D_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<reseq_interfaces::srv::BatchDetections2D_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      reseq_interfaces::srv::BatchDetections2D_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<reseq_interfaces::srv::BatchDetections2D_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      reseq_interfaces::srv::BatchDetections2D_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<reseq_interfaces::srv::BatchDetections2D_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<reseq_interfaces::srv::BatchDetections2D_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<reseq_interfaces::srv::BatchDetections2D_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__reseq_interfaces__srv__BatchDetections2D_Response
    std::shared_ptr<reseq_interfaces::srv::BatchDetections2D_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__reseq_interfaces__srv__BatchDetections2D_Response
    std::shared_ptr<reseq_interfaces::srv::BatchDetections2D_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BatchDetections2D_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const BatchDetections2D_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BatchDetections2D_Response_

// alias to use template instance with default allocator
using BatchDetections2D_Response =
  reseq_interfaces::srv::BatchDetections2D_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace reseq_interfaces


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__reseq_interfaces__srv__BatchDetections2D_Event __attribute__((deprecated))
#else
# define DEPRECATED__reseq_interfaces__srv__BatchDetections2D_Event __declspec(deprecated)
#endif

namespace reseq_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct BatchDetections2D_Event_
{
  using Type = BatchDetections2D_Event_<ContainerAllocator>;

  explicit BatchDetections2D_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit BatchDetections2D_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<reseq_interfaces::srv::BatchDetections2D_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<reseq_interfaces::srv::BatchDetections2D_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<reseq_interfaces::srv::BatchDetections2D_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<reseq_interfaces::srv::BatchDetections2D_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<reseq_interfaces::srv::BatchDetections2D_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<reseq_interfaces::srv::BatchDetections2D_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<reseq_interfaces::srv::BatchDetections2D_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<reseq_interfaces::srv::BatchDetections2D_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    reseq_interfaces::srv::BatchDetections2D_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const reseq_interfaces::srv::BatchDetections2D_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<reseq_interfaces::srv::BatchDetections2D_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<reseq_interfaces::srv::BatchDetections2D_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      reseq_interfaces::srv::BatchDetections2D_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<reseq_interfaces::srv::BatchDetections2D_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      reseq_interfaces::srv::BatchDetections2D_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<reseq_interfaces::srv::BatchDetections2D_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<reseq_interfaces::srv::BatchDetections2D_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<reseq_interfaces::srv::BatchDetections2D_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__reseq_interfaces__srv__BatchDetections2D_Event
    std::shared_ptr<reseq_interfaces::srv::BatchDetections2D_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__reseq_interfaces__srv__BatchDetections2D_Event
    std::shared_ptr<reseq_interfaces::srv::BatchDetections2D_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BatchDetections2D_Event_ & other) const
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
  bool operator!=(const BatchDetections2D_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BatchDetections2D_Event_

// alias to use template instance with default allocator
using BatchDetections2D_Event =
  reseq_interfaces::srv::BatchDetections2D_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace reseq_interfaces

namespace reseq_interfaces
{

namespace srv
{

struct BatchDetections2D
{
  using Request = reseq_interfaces::srv::BatchDetections2D_Request;
  using Response = reseq_interfaces::srv::BatchDetections2D_Response;
  using Event = reseq_interfaces::srv::BatchDetections2D_Event;
};

}  // namespace srv

}  // namespace reseq_interfaces

#endif  // RESEQ_INTERFACES__SRV__DETAIL__BATCH_DETECTIONS2_D__STRUCT_HPP_
