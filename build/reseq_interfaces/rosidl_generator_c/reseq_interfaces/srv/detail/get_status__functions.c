// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from reseq_interfaces:srv/GetStatus.idl
// generated code does not contain a copyright notice
#include "reseq_interfaces/srv/detail/get_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
reseq_interfaces__srv__GetStatus_Request__init(reseq_interfaces__srv__GetStatus_Request * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
reseq_interfaces__srv__GetStatus_Request__fini(reseq_interfaces__srv__GetStatus_Request * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
reseq_interfaces__srv__GetStatus_Request__are_equal(const reseq_interfaces__srv__GetStatus_Request * lhs, const reseq_interfaces__srv__GetStatus_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
reseq_interfaces__srv__GetStatus_Request__copy(
  const reseq_interfaces__srv__GetStatus_Request * input,
  reseq_interfaces__srv__GetStatus_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

reseq_interfaces__srv__GetStatus_Request *
reseq_interfaces__srv__GetStatus_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  reseq_interfaces__srv__GetStatus_Request * msg = (reseq_interfaces__srv__GetStatus_Request *)allocator.allocate(sizeof(reseq_interfaces__srv__GetStatus_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(reseq_interfaces__srv__GetStatus_Request));
  bool success = reseq_interfaces__srv__GetStatus_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
reseq_interfaces__srv__GetStatus_Request__destroy(reseq_interfaces__srv__GetStatus_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    reseq_interfaces__srv__GetStatus_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
reseq_interfaces__srv__GetStatus_Request__Sequence__init(reseq_interfaces__srv__GetStatus_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  reseq_interfaces__srv__GetStatus_Request * data = NULL;

  if (size) {
    data = (reseq_interfaces__srv__GetStatus_Request *)allocator.zero_allocate(size, sizeof(reseq_interfaces__srv__GetStatus_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = reseq_interfaces__srv__GetStatus_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        reseq_interfaces__srv__GetStatus_Request__fini(&data[i - 1]);
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
reseq_interfaces__srv__GetStatus_Request__Sequence__fini(reseq_interfaces__srv__GetStatus_Request__Sequence * array)
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
      reseq_interfaces__srv__GetStatus_Request__fini(&array->data[i]);
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

reseq_interfaces__srv__GetStatus_Request__Sequence *
reseq_interfaces__srv__GetStatus_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  reseq_interfaces__srv__GetStatus_Request__Sequence * array = (reseq_interfaces__srv__GetStatus_Request__Sequence *)allocator.allocate(sizeof(reseq_interfaces__srv__GetStatus_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = reseq_interfaces__srv__GetStatus_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
reseq_interfaces__srv__GetStatus_Request__Sequence__destroy(reseq_interfaces__srv__GetStatus_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    reseq_interfaces__srv__GetStatus_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
reseq_interfaces__srv__GetStatus_Request__Sequence__are_equal(const reseq_interfaces__srv__GetStatus_Request__Sequence * lhs, const reseq_interfaces__srv__GetStatus_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!reseq_interfaces__srv__GetStatus_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
reseq_interfaces__srv__GetStatus_Request__Sequence__copy(
  const reseq_interfaces__srv__GetStatus_Request__Sequence * input,
  reseq_interfaces__srv__GetStatus_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(reseq_interfaces__srv__GetStatus_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    reseq_interfaces__srv__GetStatus_Request * data =
      (reseq_interfaces__srv__GetStatus_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!reseq_interfaces__srv__GetStatus_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          reseq_interfaces__srv__GetStatus_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!reseq_interfaces__srv__GetStatus_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `csv_path`
// Member `last_error`
#include "rosidl_runtime_c/string_functions.h"

bool
reseq_interfaces__srv__GetStatus_Response__init(reseq_interfaces__srv__GetStatus_Response * msg)
{
  if (!msg) {
    return false;
  }
  // current_mode
  // initialized
  // csv_path
  if (!rosidl_runtime_c__String__init(&msg->csv_path)) {
    reseq_interfaces__srv__GetStatus_Response__fini(msg);
    return false;
  }
  // last_error
  if (!rosidl_runtime_c__String__init(&msg->last_error)) {
    reseq_interfaces__srv__GetStatus_Response__fini(msg);
    return false;
  }
  return true;
}

void
reseq_interfaces__srv__GetStatus_Response__fini(reseq_interfaces__srv__GetStatus_Response * msg)
{
  if (!msg) {
    return;
  }
  // current_mode
  // initialized
  // csv_path
  rosidl_runtime_c__String__fini(&msg->csv_path);
  // last_error
  rosidl_runtime_c__String__fini(&msg->last_error);
}

bool
reseq_interfaces__srv__GetStatus_Response__are_equal(const reseq_interfaces__srv__GetStatus_Response * lhs, const reseq_interfaces__srv__GetStatus_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // current_mode
  if (lhs->current_mode != rhs->current_mode) {
    return false;
  }
  // initialized
  if (lhs->initialized != rhs->initialized) {
    return false;
  }
  // csv_path
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->csv_path), &(rhs->csv_path)))
  {
    return false;
  }
  // last_error
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->last_error), &(rhs->last_error)))
  {
    return false;
  }
  return true;
}

bool
reseq_interfaces__srv__GetStatus_Response__copy(
  const reseq_interfaces__srv__GetStatus_Response * input,
  reseq_interfaces__srv__GetStatus_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // current_mode
  output->current_mode = input->current_mode;
  // initialized
  output->initialized = input->initialized;
  // csv_path
  if (!rosidl_runtime_c__String__copy(
      &(input->csv_path), &(output->csv_path)))
  {
    return false;
  }
  // last_error
  if (!rosidl_runtime_c__String__copy(
      &(input->last_error), &(output->last_error)))
  {
    return false;
  }
  return true;
}

reseq_interfaces__srv__GetStatus_Response *
reseq_interfaces__srv__GetStatus_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  reseq_interfaces__srv__GetStatus_Response * msg = (reseq_interfaces__srv__GetStatus_Response *)allocator.allocate(sizeof(reseq_interfaces__srv__GetStatus_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(reseq_interfaces__srv__GetStatus_Response));
  bool success = reseq_interfaces__srv__GetStatus_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
reseq_interfaces__srv__GetStatus_Response__destroy(reseq_interfaces__srv__GetStatus_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    reseq_interfaces__srv__GetStatus_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
reseq_interfaces__srv__GetStatus_Response__Sequence__init(reseq_interfaces__srv__GetStatus_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  reseq_interfaces__srv__GetStatus_Response * data = NULL;

  if (size) {
    data = (reseq_interfaces__srv__GetStatus_Response *)allocator.zero_allocate(size, sizeof(reseq_interfaces__srv__GetStatus_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = reseq_interfaces__srv__GetStatus_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        reseq_interfaces__srv__GetStatus_Response__fini(&data[i - 1]);
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
reseq_interfaces__srv__GetStatus_Response__Sequence__fini(reseq_interfaces__srv__GetStatus_Response__Sequence * array)
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
      reseq_interfaces__srv__GetStatus_Response__fini(&array->data[i]);
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

reseq_interfaces__srv__GetStatus_Response__Sequence *
reseq_interfaces__srv__GetStatus_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  reseq_interfaces__srv__GetStatus_Response__Sequence * array = (reseq_interfaces__srv__GetStatus_Response__Sequence *)allocator.allocate(sizeof(reseq_interfaces__srv__GetStatus_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = reseq_interfaces__srv__GetStatus_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
reseq_interfaces__srv__GetStatus_Response__Sequence__destroy(reseq_interfaces__srv__GetStatus_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    reseq_interfaces__srv__GetStatus_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
reseq_interfaces__srv__GetStatus_Response__Sequence__are_equal(const reseq_interfaces__srv__GetStatus_Response__Sequence * lhs, const reseq_interfaces__srv__GetStatus_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!reseq_interfaces__srv__GetStatus_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
reseq_interfaces__srv__GetStatus_Response__Sequence__copy(
  const reseq_interfaces__srv__GetStatus_Response__Sequence * input,
  reseq_interfaces__srv__GetStatus_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(reseq_interfaces__srv__GetStatus_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    reseq_interfaces__srv__GetStatus_Response * data =
      (reseq_interfaces__srv__GetStatus_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!reseq_interfaces__srv__GetStatus_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          reseq_interfaces__srv__GetStatus_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!reseq_interfaces__srv__GetStatus_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `info`
#include "service_msgs/msg/detail/service_event_info__functions.h"
// Member `request`
// Member `response`
// already included above
// #include "reseq_interfaces/srv/detail/get_status__functions.h"

bool
reseq_interfaces__srv__GetStatus_Event__init(reseq_interfaces__srv__GetStatus_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    reseq_interfaces__srv__GetStatus_Event__fini(msg);
    return false;
  }
  // request
  if (!reseq_interfaces__srv__GetStatus_Request__Sequence__init(&msg->request, 0)) {
    reseq_interfaces__srv__GetStatus_Event__fini(msg);
    return false;
  }
  // response
  if (!reseq_interfaces__srv__GetStatus_Response__Sequence__init(&msg->response, 0)) {
    reseq_interfaces__srv__GetStatus_Event__fini(msg);
    return false;
  }
  return true;
}

void
reseq_interfaces__srv__GetStatus_Event__fini(reseq_interfaces__srv__GetStatus_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  reseq_interfaces__srv__GetStatus_Request__Sequence__fini(&msg->request);
  // response
  reseq_interfaces__srv__GetStatus_Response__Sequence__fini(&msg->response);
}

bool
reseq_interfaces__srv__GetStatus_Event__are_equal(const reseq_interfaces__srv__GetStatus_Event * lhs, const reseq_interfaces__srv__GetStatus_Event * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // request
  if (!reseq_interfaces__srv__GetStatus_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!reseq_interfaces__srv__GetStatus_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
reseq_interfaces__srv__GetStatus_Event__copy(
  const reseq_interfaces__srv__GetStatus_Event * input,
  reseq_interfaces__srv__GetStatus_Event * output)
{
  if (!input || !output) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // request
  if (!reseq_interfaces__srv__GetStatus_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!reseq_interfaces__srv__GetStatus_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

reseq_interfaces__srv__GetStatus_Event *
reseq_interfaces__srv__GetStatus_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  reseq_interfaces__srv__GetStatus_Event * msg = (reseq_interfaces__srv__GetStatus_Event *)allocator.allocate(sizeof(reseq_interfaces__srv__GetStatus_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(reseq_interfaces__srv__GetStatus_Event));
  bool success = reseq_interfaces__srv__GetStatus_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
reseq_interfaces__srv__GetStatus_Event__destroy(reseq_interfaces__srv__GetStatus_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    reseq_interfaces__srv__GetStatus_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
reseq_interfaces__srv__GetStatus_Event__Sequence__init(reseq_interfaces__srv__GetStatus_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  reseq_interfaces__srv__GetStatus_Event * data = NULL;

  if (size) {
    data = (reseq_interfaces__srv__GetStatus_Event *)allocator.zero_allocate(size, sizeof(reseq_interfaces__srv__GetStatus_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = reseq_interfaces__srv__GetStatus_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        reseq_interfaces__srv__GetStatus_Event__fini(&data[i - 1]);
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
reseq_interfaces__srv__GetStatus_Event__Sequence__fini(reseq_interfaces__srv__GetStatus_Event__Sequence * array)
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
      reseq_interfaces__srv__GetStatus_Event__fini(&array->data[i]);
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

reseq_interfaces__srv__GetStatus_Event__Sequence *
reseq_interfaces__srv__GetStatus_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  reseq_interfaces__srv__GetStatus_Event__Sequence * array = (reseq_interfaces__srv__GetStatus_Event__Sequence *)allocator.allocate(sizeof(reseq_interfaces__srv__GetStatus_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = reseq_interfaces__srv__GetStatus_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
reseq_interfaces__srv__GetStatus_Event__Sequence__destroy(reseq_interfaces__srv__GetStatus_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    reseq_interfaces__srv__GetStatus_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
reseq_interfaces__srv__GetStatus_Event__Sequence__are_equal(const reseq_interfaces__srv__GetStatus_Event__Sequence * lhs, const reseq_interfaces__srv__GetStatus_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!reseq_interfaces__srv__GetStatus_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
reseq_interfaces__srv__GetStatus_Event__Sequence__copy(
  const reseq_interfaces__srv__GetStatus_Event__Sequence * input,
  reseq_interfaces__srv__GetStatus_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(reseq_interfaces__srv__GetStatus_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    reseq_interfaces__srv__GetStatus_Event * data =
      (reseq_interfaces__srv__GetStatus_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!reseq_interfaces__srv__GetStatus_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          reseq_interfaces__srv__GetStatus_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!reseq_interfaces__srv__GetStatus_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
