// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from reseq_interfaces:msg/EndEffector.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "reseq_interfaces/msg/end_effector.hpp"


#ifndef RESEQ_INTERFACES__MSG__DETAIL__END_EFFECTOR__STRUCT_HPP_
#define RESEQ_INTERFACES__MSG__DETAIL__END_EFFECTOR__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__reseq_interfaces__msg__EndEffector __attribute__((deprecated))
#else
# define DEPRECATED__reseq_interfaces__msg__EndEffector __declspec(deprecated)
#endif

namespace reseq_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct EndEffector_
{
  using Type = EndEffector_<ContainerAllocator>;

  explicit EndEffector_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pitch_vel = 0.0f;
      this->head_pitch_vel = 0.0f;
      this->head_roll_vel = 0.0f;
    }
  }

  explicit EndEffector_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pitch_vel = 0.0f;
      this->head_pitch_vel = 0.0f;
      this->head_roll_vel = 0.0f;
    }
  }

  // field types and members
  using _pitch_vel_type =
    float;
  _pitch_vel_type pitch_vel;
  using _head_pitch_vel_type =
    float;
  _head_pitch_vel_type head_pitch_vel;
  using _head_roll_vel_type =
    float;
  _head_roll_vel_type head_roll_vel;

  // setters for named parameter idiom
  Type & set__pitch_vel(
    const float & _arg)
  {
    this->pitch_vel = _arg;
    return *this;
  }
  Type & set__head_pitch_vel(
    const float & _arg)
  {
    this->head_pitch_vel = _arg;
    return *this;
  }
  Type & set__head_roll_vel(
    const float & _arg)
  {
    this->head_roll_vel = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    reseq_interfaces::msg::EndEffector_<ContainerAllocator> *;
  using ConstRawPtr =
    const reseq_interfaces::msg::EndEffector_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<reseq_interfaces::msg::EndEffector_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<reseq_interfaces::msg::EndEffector_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      reseq_interfaces::msg::EndEffector_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<reseq_interfaces::msg::EndEffector_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      reseq_interfaces::msg::EndEffector_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<reseq_interfaces::msg::EndEffector_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<reseq_interfaces::msg::EndEffector_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<reseq_interfaces::msg::EndEffector_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__reseq_interfaces__msg__EndEffector
    std::shared_ptr<reseq_interfaces::msg::EndEffector_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__reseq_interfaces__msg__EndEffector
    std::shared_ptr<reseq_interfaces::msg::EndEffector_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const EndEffector_ & other) const
  {
    if (this->pitch_vel != other.pitch_vel) {
      return false;
    }
    if (this->head_pitch_vel != other.head_pitch_vel) {
      return false;
    }
    if (this->head_roll_vel != other.head_roll_vel) {
      return false;
    }
    return true;
  }
  bool operator!=(const EndEffector_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct EndEffector_

// alias to use template instance with default allocator
using EndEffector =
  reseq_interfaces::msg::EndEffector_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace reseq_interfaces

#endif  // RESEQ_INTERFACES__MSG__DETAIL__END_EFFECTOR__STRUCT_HPP_
