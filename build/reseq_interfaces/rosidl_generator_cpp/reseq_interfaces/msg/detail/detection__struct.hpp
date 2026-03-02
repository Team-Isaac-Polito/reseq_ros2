// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from reseq_interfaces:msg/Detection.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "reseq_interfaces/msg/detection.hpp"


#ifndef RESEQ_INTERFACES__MSG__DETAIL__DETECTION__STRUCT_HPP_
#define RESEQ_INTERFACES__MSG__DETAIL__DETECTION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__reseq_interfaces__msg__Detection __attribute__((deprecated))
#else
# define DEPRECATED__reseq_interfaces__msg__Detection __declspec(deprecated)
#endif

namespace reseq_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Detection_
{
  using Type = Detection_<ContainerAllocator>;

  explicit Detection_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->detection = 0l;
      this->type = "";
      this->name = "";
      this->robot = "";
      this->mode = "";
      this->confidence = 0.0f;
      this->xmin = 0l;
      this->ymin = 0l;
      this->width = 0ul;
      this->height = 0ul;
      this->depth_center = 0.0f;
      this->camera_frame = "";
    }
  }

  explicit Detection_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    type(_alloc),
    name(_alloc),
    robot(_alloc),
    mode(_alloc),
    camera_frame(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->detection = 0l;
      this->type = "";
      this->name = "";
      this->robot = "";
      this->mode = "";
      this->confidence = 0.0f;
      this->xmin = 0l;
      this->ymin = 0l;
      this->width = 0ul;
      this->height = 0ul;
      this->depth_center = 0.0f;
      this->camera_frame = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _detection_type =
    int32_t;
  _detection_type detection;
  using _type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _type_type type;
  using _name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _name_type name;
  using _robot_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _robot_type robot;
  using _mode_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _mode_type mode;
  using _confidence_type =
    float;
  _confidence_type confidence;
  using _xmin_type =
    int32_t;
  _xmin_type xmin;
  using _ymin_type =
    int32_t;
  _ymin_type ymin;
  using _width_type =
    uint32_t;
  _width_type width;
  using _height_type =
    uint32_t;
  _height_type height;
  using _depth_center_type =
    float;
  _depth_center_type depth_center;
  using _camera_frame_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _camera_frame_type camera_frame;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__detection(
    const int32_t & _arg)
  {
    this->detection = _arg;
    return *this;
  }
  Type & set__type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->type = _arg;
    return *this;
  }
  Type & set__name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->name = _arg;
    return *this;
  }
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
  Type & set__confidence(
    const float & _arg)
  {
    this->confidence = _arg;
    return *this;
  }
  Type & set__xmin(
    const int32_t & _arg)
  {
    this->xmin = _arg;
    return *this;
  }
  Type & set__ymin(
    const int32_t & _arg)
  {
    this->ymin = _arg;
    return *this;
  }
  Type & set__width(
    const uint32_t & _arg)
  {
    this->width = _arg;
    return *this;
  }
  Type & set__height(
    const uint32_t & _arg)
  {
    this->height = _arg;
    return *this;
  }
  Type & set__depth_center(
    const float & _arg)
  {
    this->depth_center = _arg;
    return *this;
  }
  Type & set__camera_frame(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->camera_frame = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    reseq_interfaces::msg::Detection_<ContainerAllocator> *;
  using ConstRawPtr =
    const reseq_interfaces::msg::Detection_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<reseq_interfaces::msg::Detection_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<reseq_interfaces::msg::Detection_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      reseq_interfaces::msg::Detection_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<reseq_interfaces::msg::Detection_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      reseq_interfaces::msg::Detection_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<reseq_interfaces::msg::Detection_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<reseq_interfaces::msg::Detection_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<reseq_interfaces::msg::Detection_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__reseq_interfaces__msg__Detection
    std::shared_ptr<reseq_interfaces::msg::Detection_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__reseq_interfaces__msg__Detection
    std::shared_ptr<reseq_interfaces::msg::Detection_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Detection_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->detection != other.detection) {
      return false;
    }
    if (this->type != other.type) {
      return false;
    }
    if (this->name != other.name) {
      return false;
    }
    if (this->robot != other.robot) {
      return false;
    }
    if (this->mode != other.mode) {
      return false;
    }
    if (this->confidence != other.confidence) {
      return false;
    }
    if (this->xmin != other.xmin) {
      return false;
    }
    if (this->ymin != other.ymin) {
      return false;
    }
    if (this->width != other.width) {
      return false;
    }
    if (this->height != other.height) {
      return false;
    }
    if (this->depth_center != other.depth_center) {
      return false;
    }
    if (this->camera_frame != other.camera_frame) {
      return false;
    }
    return true;
  }
  bool operator!=(const Detection_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Detection_

// alias to use template instance with default allocator
using Detection =
  reseq_interfaces::msg::Detection_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace reseq_interfaces

#endif  // RESEQ_INTERFACES__MSG__DETAIL__DETECTION__STRUCT_HPP_
