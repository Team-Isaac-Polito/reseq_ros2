// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from reseq_interfaces:msg/Remote.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "reseq_interfaces/msg/remote.hpp"


#ifndef RESEQ_INTERFACES__MSG__DETAIL__REMOTE__STRUCT_HPP_
#define RESEQ_INTERFACES__MSG__DETAIL__REMOTE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'left'
// Member 'right'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__reseq_interfaces__msg__Remote __attribute__((deprecated))
#else
# define DEPRECATED__reseq_interfaces__msg__Remote __declspec(deprecated)
#endif

namespace reseq_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Remote_
{
  using Type = Remote_<ContainerAllocator>;

  explicit Remote_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : left(_init),
    right(_init)
  {
    (void)_init;
  }

  explicit Remote_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : left(_alloc, _init),
    right(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _left_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _left_type left;
  using _right_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _right_type right;
  using _buttons_type =
    std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>>;
  _buttons_type buttons;
  using _switches_type =
    std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>>;
  _switches_type switches;

  // setters for named parameter idiom
  Type & set__left(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->left = _arg;
    return *this;
  }
  Type & set__right(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->right = _arg;
    return *this;
  }
  Type & set__buttons(
    const std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>> & _arg)
  {
    this->buttons = _arg;
    return *this;
  }
  Type & set__switches(
    const std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>> & _arg)
  {
    this->switches = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    reseq_interfaces::msg::Remote_<ContainerAllocator> *;
  using ConstRawPtr =
    const reseq_interfaces::msg::Remote_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<reseq_interfaces::msg::Remote_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<reseq_interfaces::msg::Remote_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      reseq_interfaces::msg::Remote_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<reseq_interfaces::msg::Remote_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      reseq_interfaces::msg::Remote_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<reseq_interfaces::msg::Remote_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<reseq_interfaces::msg::Remote_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<reseq_interfaces::msg::Remote_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__reseq_interfaces__msg__Remote
    std::shared_ptr<reseq_interfaces::msg::Remote_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__reseq_interfaces__msg__Remote
    std::shared_ptr<reseq_interfaces::msg::Remote_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Remote_ & other) const
  {
    if (this->left != other.left) {
      return false;
    }
    if (this->right != other.right) {
      return false;
    }
    if (this->buttons != other.buttons) {
      return false;
    }
    if (this->switches != other.switches) {
      return false;
    }
    return true;
  }
  bool operator!=(const Remote_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Remote_

// alias to use template instance with default allocator
using Remote =
  reseq_interfaces::msg::Remote_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace reseq_interfaces

#endif  // RESEQ_INTERFACES__MSG__DETAIL__REMOTE__STRUCT_HPP_
