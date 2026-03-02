// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from reseq_interfaces:msg/EndEffector.idl
// generated code does not contain a copyright notice
#include "reseq_interfaces/msg/detail/end_effector__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
reseq_interfaces__msg__EndEffector__init(reseq_interfaces__msg__EndEffector * msg)
{
  if (!msg) {
    return false;
  }
  // pitch_vel
  // head_pitch_vel
  // head_roll_vel
  return true;
}

void
reseq_interfaces__msg__EndEffector__fini(reseq_interfaces__msg__EndEffector * msg)
{
  if (!msg) {
    return;
  }
  // pitch_vel
  // head_pitch_vel
  // head_roll_vel
}

bool
reseq_interfaces__msg__EndEffector__are_equal(const reseq_interfaces__msg__EndEffector * lhs, const reseq_interfaces__msg__EndEffector * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // pitch_vel
  if (lhs->pitch_vel != rhs->pitch_vel) {
    return false;
  }
  // head_pitch_vel
  if (lhs->head_pitch_vel != rhs->head_pitch_vel) {
    return false;
  }
  // head_roll_vel
  if (lhs->head_roll_vel != rhs->head_roll_vel) {
    return false;
  }
  return true;
}

bool
reseq_interfaces__msg__EndEffector__copy(
  const reseq_interfaces__msg__EndEffector * input,
  reseq_interfaces__msg__EndEffector * output)
{
  if (!input || !output) {
    return false;
  }
  // pitch_vel
  output->pitch_vel = input->pitch_vel;
  // head_pitch_vel
  output->head_pitch_vel = input->head_pitch_vel;
  // head_roll_vel
  output->head_roll_vel = input->head_roll_vel;
  return true;
}

reseq_interfaces__msg__EndEffector *
reseq_interfaces__msg__EndEffector__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  reseq_interfaces__msg__EndEffector * msg = (reseq_interfaces__msg__EndEffector *)allocator.allocate(sizeof(reseq_interfaces__msg__EndEffector), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(reseq_interfaces__msg__EndEffector));
  bool success = reseq_interfaces__msg__EndEffector__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
reseq_interfaces__msg__EndEffector__destroy(reseq_interfaces__msg__EndEffector * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    reseq_interfaces__msg__EndEffector__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
reseq_interfaces__msg__EndEffector__Sequence__init(reseq_interfaces__msg__EndEffector__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  reseq_interfaces__msg__EndEffector * data = NULL;

  if (size) {
    data = (reseq_interfaces__msg__EndEffector *)allocator.zero_allocate(size, sizeof(reseq_interfaces__msg__EndEffector), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = reseq_interfaces__msg__EndEffector__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        reseq_interfaces__msg__EndEffector__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
reseq_interfaces__msg__EndEffector__Sequence__fini(reseq_interfaces__msg__EndEffector__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      reseq_interfaces__msg__EndEffector__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

reseq_interfaces__msg__EndEffector__Sequence *
reseq_interfaces__msg__EndEffector__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  reseq_interfaces__msg__EndEffector__Sequence * array = (reseq_interfaces__msg__EndEffector__Sequence *)allocator.allocate(sizeof(reseq_interfaces__msg__EndEffector__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = reseq_interfaces__msg__EndEffector__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
reseq_interfaces__msg__EndEffector__Sequence__destroy(reseq_interfaces__msg__EndEffector__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    reseq_interfaces__msg__EndEffector__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
reseq_interfaces__msg__EndEffector__Sequence__are_equal(const reseq_interfaces__msg__EndEffector__Sequence * lhs, const reseq_interfaces__msg__EndEffector__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!reseq_interfaces__msg__EndEffector__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
reseq_interfaces__msg__EndEffector__Sequence__copy(
  const reseq_interfaces__msg__EndEffector__Sequence * input,
  reseq_interfaces__msg__EndEffector__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(reseq_interfaces__msg__EndEffector);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    reseq_interfaces__msg__EndEffector * data =
      (reseq_interfaces__msg__EndEffector *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!reseq_interfaces__msg__EndEffector__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          reseq_interfaces__msg__EndEffector__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!reseq_interfaces__msg__EndEffector__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
