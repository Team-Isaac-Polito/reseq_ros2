// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from reseq_interfaces:msg/Motors.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "reseq_interfaces/msg/motors.hpp"


#ifndef RESEQ_INTERFACES__MSG__DETAIL__MOTORS__STRUCT_HPP_
#define RESEQ_INTERFACES__MSG__DETAIL__MOTORS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__reseq_interfaces__msg__Motors __attribute__((deprecated))
#else
# define DEPRECATED__reseq_interfaces__msg__Motors __declspec(deprecated)
#endif

namespace reseq_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Motors_
{
  using Type = Motors_<ContainerAllocator>;

  explicit Motors_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->left = 0.0f;
      this->right = 0.0f;
    }
  }

  explicit Motors_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->left = 0.0f;
      this->right = 0.0f;
    }
  }

  // field types and members
  using _left_type =
    float;
  _left_type left;
  using _right_type =
    float;
  _right_type right;

  // setters for named parameter idiom
  Type & set__left(
    const float & _arg)
  {
    this->left = _arg;
    return *this;
  }
  Type & set__right(
    const float & _arg)
  {
    this->right = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    reseq_interfaces::msg::Motors_<ContainerAllocator> *;
  using ConstRawPtr =
    const reseq_interfaces::msg::Motors_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<reseq_interfaces::msg::Motors_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<reseq_interfaces::msg::Motors_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      reseq_interfaces::msg::Motors_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<reseq_interfaces::msg::Motors_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      reseq_interfaces::msg::Motors_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<reseq_interfaces::msg::Motors_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<reseq_interfaces::msg::Motors_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<reseq_interfaces::msg::Motors_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__reseq_interfaces__msg__Motors
    std::shared_ptr<reseq_interfaces::msg::Motors_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__reseq_interfaces__msg__Motors
    std::shared_ptr<reseq_interfaces::msg::Motors_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Motors_ & other) const
  {
    if (this->left != other.left) {
      return false;
    }
    if (this->right != other.right) {
      return false;
    }
    return true;
  }
  bool operator!=(const Motors_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Motors_

// alias to use template instance with default allocator
using Motors =
  reseq_interfaces::msg::Motors_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace reseq_interfaces

#endif  // RESEQ_INTERFACES__MSG__DETAIL__MOTORS__STRUCT_HPP_
