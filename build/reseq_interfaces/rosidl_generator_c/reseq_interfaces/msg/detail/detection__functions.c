// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from reseq_interfaces:msg/Detection.idl
// generated code does not contain a copyright notice
#include "reseq_interfaces/msg/detail/detection__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `type`
// Member `name`
// Member `robot`
// Member `mode`
// Member `camera_frame`
#include "rosidl_runtime_c/string_functions.h"

bool
reseq_interfaces__msg__Detection__init(reseq_interfaces__msg__Detection * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    reseq_interfaces__msg__Detection__fini(msg);
    return false;
  }
  // detection
  // type
  if (!rosidl_runtime_c__String__init(&msg->type)) {
    reseq_interfaces__msg__Detection__fini(msg);
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__init(&msg->name)) {
    reseq_interfaces__msg__Detection__fini(msg);
    return false;
  }
  // robot
  if (!rosidl_runtime_c__String__init(&msg->robot)) {
    reseq_interfaces__msg__Detection__fini(msg);
    return false;
  }
  // mode
  if (!rosidl_runtime_c__String__init(&msg->mode)) {
    reseq_interfaces__msg__Detection__fini(msg);
    return false;
  }
  // confidence
  // xmin
  // ymin
  // width
  // height
  // depth_center
  // camera_frame
  if (!rosidl_runtime_c__String__init(&msg->camera_frame)) {
    reseq_interfaces__msg__Detection__fini(msg);
    return false;
  }
  return true;
}

void
reseq_interfaces__msg__Detection__fini(reseq_interfaces__msg__Detection * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // detection
  // type
  rosidl_runtime_c__String__fini(&msg->type);
  // name
  rosidl_runtime_c__String__fini(&msg->name);
  // robot
  rosidl_runtime_c__String__fini(&msg->robot);
  // mode
  rosidl_runtime_c__String__fini(&msg->mode);
  // confidence
  // xmin
  // ymin
  // width
  // height
  // depth_center
  // camera_frame
  rosidl_runtime_c__String__fini(&msg->camera_frame);
}

bool
reseq_interfaces__msg__Detection__are_equal(const reseq_interfaces__msg__Detection * lhs, const reseq_interfaces__msg__Detection * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // detection
  if (lhs->detection != rhs->detection) {
    return false;
  }
  // type
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->type), &(rhs->type)))
  {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->name), &(rhs->name)))
  {
    return false;
  }
  // robot
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->robot), &(rhs->robot)))
  {
    return false;
  }
  // mode
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->mode), &(rhs->mode)))
  {
    return false;
  }
  // confidence
  if (lhs->confidence != rhs->confidence) {
    return false;
  }
  // xmin
  if (lhs->xmin != rhs->xmin) {
    return false;
  }
  // ymin
  if (lhs->ymin != rhs->ymin) {
    return false;
  }
  // width
  if (lhs->width != rhs->width) {
    return false;
  }
  // height
  if (lhs->height != rhs->height) {
    return false;
  }
  // depth_center
  if (lhs->depth_center != rhs->depth_center) {
    return false;
  }
  // camera_frame
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->camera_frame), &(rhs->camera_frame)))
  {
    return false;
  }
  return true;
}

bool
reseq_interfaces__msg__Detection__copy(
  const reseq_interfaces__msg__Detection * input,
  reseq_interfaces__msg__Detection * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // detection
  output->detection = input->detection;
  // type
  if (!rosidl_runtime_c__String__copy(
      &(input->type), &(output->type)))
  {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__copy(
      &(input->name), &(output->name)))
  {
    return false;
  }
  // robot
  if (!rosidl_runtime_c__String__copy(
      &(input->robot), &(output->robot)))
  {
    return false;
  }
  // mode
  if (!rosidl_runtime_c__String__copy(
      &(input->mode), &(output->mode)))
  {
    return false;
  }
  // confidence
  output->confidence = input->confidence;
  // xmin
  output->xmin = input->xmin;
  // ymin
  output->ymin = input->ymin;
  // width
  output->width = input->width;
  // height
  output->height = input->height;
  // depth_center
  output->depth_center = input->depth_center;
  // camera_frame
  if (!rosidl_runtime_c__String__copy(
      &(input->camera_frame), &(output->camera_frame)))
  {
    return false;
  }
  return true;
}

reseq_interfaces__msg__Detection *
reseq_interfaces__msg__Detection__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  reseq_interfaces__msg__Detection * msg = (reseq_interfaces__msg__Detection *)allocator.allocate(sizeof(reseq_interfaces__msg__Detection), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(reseq_interfaces__msg__Detection));
  bool success = reseq_interfaces__msg__Detection__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
reseq_interfaces__msg__Detection__destroy(reseq_interfaces__msg__Detection * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    reseq_interfaces__msg__Detection__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
reseq_interfaces__msg__Detection__Sequence__init(reseq_interfaces__msg__Detection__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  reseq_interfaces__msg__Detection * data = NULL;

  if (size) {
    data = (reseq_interfaces__msg__Detection *)allocator.zero_allocate(size, sizeof(reseq_interfaces__msg__Detection), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = reseq_interfaces__msg__Detection__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        reseq_interfaces__msg__Detection__fini(&data[i - 1]);
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
reseq_interfaces__msg__Detection__Sequence__fini(reseq_interfaces__msg__Detection__Sequence * array)
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
      reseq_interfaces__msg__Detection__fini(&array->data[i]);
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

reseq_interfaces__msg__Detection__Sequence *
reseq_interfaces__msg__Detection__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  reseq_interfaces__msg__Detection__Sequence * array = (reseq_interfaces__msg__Detection__Sequence *)allocator.allocate(sizeof(reseq_interfaces__msg__Detection__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = reseq_interfaces__msg__Detection__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
reseq_interfaces__msg__Detection__Sequence__destroy(reseq_interfaces__msg__Detection__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    reseq_interfaces__msg__Detection__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
reseq_interfaces__msg__Detection__Sequence__are_equal(const reseq_interfaces__msg__Detection__Sequence * lhs, const reseq_interfaces__msg__Detection__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!reseq_interfaces__msg__Detection__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
reseq_interfaces__msg__Detection__Sequence__copy(
  const reseq_interfaces__msg__Detection__Sequence * input,
  reseq_interfaces__msg__Detection__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(reseq_interfaces__msg__Detection);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    reseq_interfaces__msg__Detection * data =
      (reseq_interfaces__msg__Detection *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!reseq_interfaces__msg__Detection__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          reseq_interfaces__msg__Detection__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!reseq_interfaces__msg__Detection__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
