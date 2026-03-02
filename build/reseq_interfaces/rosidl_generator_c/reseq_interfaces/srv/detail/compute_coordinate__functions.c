// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from reseq_interfaces:srv/ComputeCoordinate.idl
// generated code does not contain a copyright notice
#include "reseq_interfaces/srv/detail/compute_coordinate__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `detection`
#include "reseq_interfaces/msg/detail/detection__functions.h"
// Member `camera_info`
#include "sensor_msgs/msg/detail/camera_info__functions.h"
// Member `target_frame`
#include "rosidl_runtime_c/string_functions.h"

bool
reseq_interfaces__srv__ComputeCoordinate_Request__init(reseq_interfaces__srv__ComputeCoordinate_Request * msg)
{
  if (!msg) {
    return false;
  }
  // detection
  if (!reseq_interfaces__msg__Detection__init(&msg->detection)) {
    reseq_interfaces__srv__ComputeCoordinate_Request__fini(msg);
    return false;
  }
  // camera_info
  if (!sensor_msgs__msg__CameraInfo__init(&msg->camera_info)) {
    reseq_interfaces__srv__ComputeCoordinate_Request__fini(msg);
    return false;
  }
  // target_frame
  if (!rosidl_runtime_c__String__init(&msg->target_frame)) {
    reseq_interfaces__srv__ComputeCoordinate_Request__fini(msg);
    return false;
  }
  return true;
}

void
reseq_interfaces__srv__ComputeCoordinate_Request__fini(reseq_interfaces__srv__ComputeCoordinate_Request * msg)
{
  if (!msg) {
    return;
  }
  // detection
  reseq_interfaces__msg__Detection__fini(&msg->detection);
  // camera_info
  sensor_msgs__msg__CameraInfo__fini(&msg->camera_info);
  // target_frame
  rosidl_runtime_c__String__fini(&msg->target_frame);
}

bool
reseq_interfaces__srv__ComputeCoordinate_Request__are_equal(const reseq_interfaces__srv__ComputeCoordinate_Request * lhs, const reseq_interfaces__srv__ComputeCoordinate_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // detection
  if (!reseq_interfaces__msg__Detection__are_equal(
      &(lhs->detection), &(rhs->detection)))
  {
    return false;
  }
  // camera_info
  if (!sensor_msgs__msg__CameraInfo__are_equal(
      &(lhs->camera_info), &(rhs->camera_info)))
  {
    return false;
  }
  // target_frame
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->target_frame), &(rhs->target_frame)))
  {
    return false;
  }
  return true;
}

bool
reseq_interfaces__srv__ComputeCoordinate_Request__copy(
  const reseq_interfaces__srv__ComputeCoordinate_Request * input,
  reseq_interfaces__srv__ComputeCoordinate_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // detection
  if (!reseq_interfaces__msg__Detection__copy(
      &(input->detection), &(output->detection)))
  {
    return false;
  }
  // camera_info
  if (!sensor_msgs__msg__CameraInfo__copy(
      &(input->camera_info), &(output->camera_info)))
  {
    return false;
  }
  // target_frame
  if (!rosidl_runtime_c__String__copy(
      &(input->target_frame), &(output->target_frame)))
  {
    return false;
  }
  return true;
}

reseq_interfaces__srv__ComputeCoordinate_Request *
reseq_interfaces__srv__ComputeCoordinate_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  reseq_interfaces__srv__ComputeCoordinate_Request * msg = (reseq_interfaces__srv__ComputeCoordinate_Request *)allocator.allocate(sizeof(reseq_interfaces__srv__ComputeCoordinate_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(reseq_interfaces__srv__ComputeCoordinate_Request));
  bool success = reseq_interfaces__srv__ComputeCoordinate_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
reseq_interfaces__srv__ComputeCoordinate_Request__destroy(reseq_interfaces__srv__ComputeCoordinate_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    reseq_interfaces__srv__ComputeCoordinate_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
reseq_interfaces__srv__ComputeCoordinate_Request__Sequence__init(reseq_interfaces__srv__ComputeCoordinate_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  reseq_interfaces__srv__ComputeCoordinate_Request * data = NULL;

  if (size) {
    data = (reseq_interfaces__srv__ComputeCoordinate_Request *)allocator.zero_allocate(size, sizeof(reseq_interfaces__srv__ComputeCoordinate_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = reseq_interfaces__srv__ComputeCoordinate_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        reseq_interfaces__srv__ComputeCoordinate_Request__fini(&data[i - 1]);
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
reseq_interfaces__srv__ComputeCoordinate_Request__Sequence__fini(reseq_interfaces__srv__ComputeCoordinate_Request__Sequence * array)
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
      reseq_interfaces__srv__ComputeCoordinate_Request__fini(&array->data[i]);
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

reseq_interfaces__srv__ComputeCoordinate_Request__Sequence *
reseq_interfaces__srv__ComputeCoordinate_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  reseq_interfaces__srv__ComputeCoordinate_Request__Sequence * array = (reseq_interfaces__srv__ComputeCoordinate_Request__Sequence *)allocator.allocate(sizeof(reseq_interfaces__srv__ComputeCoordinate_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = reseq_interfaces__srv__ComputeCoordinate_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
reseq_interfaces__srv__ComputeCoordinate_Request__Sequence__destroy(reseq_interfaces__srv__ComputeCoordinate_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    reseq_interfaces__srv__ComputeCoordinate_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
reseq_interfaces__srv__ComputeCoordinate_Request__Sequence__are_equal(const reseq_interfaces__srv__ComputeCoordinate_Request__Sequence * lhs, const reseq_interfaces__srv__ComputeCoordinate_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!reseq_interfaces__srv__ComputeCoordinate_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
reseq_interfaces__srv__ComputeCoordinate_Request__Sequence__copy(
  const reseq_interfaces__srv__ComputeCoordinate_Request__Sequence * input,
  reseq_interfaces__srv__ComputeCoordinate_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(reseq_interfaces__srv__ComputeCoordinate_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    reseq_interfaces__srv__ComputeCoordinate_Request * data =
      (reseq_interfaces__srv__ComputeCoordinate_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!reseq_interfaces__srv__ComputeCoordinate_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          reseq_interfaces__srv__ComputeCoordinate_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!reseq_interfaces__srv__ComputeCoordinate_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `point`
#include "geometry_msgs/msg/detail/point_stamped__functions.h"
// Member `message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
reseq_interfaces__srv__ComputeCoordinate_Response__init(reseq_interfaces__srv__ComputeCoordinate_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // point
  if (!geometry_msgs__msg__PointStamped__init(&msg->point)) {
    reseq_interfaces__srv__ComputeCoordinate_Response__fini(msg);
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    reseq_interfaces__srv__ComputeCoordinate_Response__fini(msg);
    return false;
  }
  return true;
}

void
reseq_interfaces__srv__ComputeCoordinate_Response__fini(reseq_interfaces__srv__ComputeCoordinate_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // point
  geometry_msgs__msg__PointStamped__fini(&msg->point);
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
reseq_interfaces__srv__ComputeCoordinate_Response__are_equal(const reseq_interfaces__srv__ComputeCoordinate_Response * lhs, const reseq_interfaces__srv__ComputeCoordinate_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // point
  if (!geometry_msgs__msg__PointStamped__are_equal(
      &(lhs->point), &(rhs->point)))
  {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  return true;
}

bool
reseq_interfaces__srv__ComputeCoordinate_Response__copy(
  const reseq_interfaces__srv__ComputeCoordinate_Response * input,
  reseq_interfaces__srv__ComputeCoordinate_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // point
  if (!geometry_msgs__msg__PointStamped__copy(
      &(input->point), &(output->point)))
  {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  return true;
}

reseq_interfaces__srv__ComputeCoordinate_Response *
reseq_interfaces__srv__ComputeCoordinate_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  reseq_interfaces__srv__ComputeCoordinate_Response * msg = (reseq_interfaces__srv__ComputeCoordinate_Response *)allocator.allocate(sizeof(reseq_interfaces__srv__ComputeCoordinate_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(reseq_interfaces__srv__ComputeCoordinate_Response));
  bool success = reseq_interfaces__srv__ComputeCoordinate_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
reseq_interfaces__srv__ComputeCoordinate_Response__destroy(reseq_interfaces__srv__ComputeCoordinate_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    reseq_interfaces__srv__ComputeCoordinate_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
reseq_interfaces__srv__ComputeCoordinate_Response__Sequence__init(reseq_interfaces__srv__ComputeCoordinate_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  reseq_interfaces__srv__ComputeCoordinate_Response * data = NULL;

  if (size) {
    data = (reseq_interfaces__srv__ComputeCoordinate_Response *)allocator.zero_allocate(size, sizeof(reseq_interfaces__srv__ComputeCoordinate_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = reseq_interfaces__srv__ComputeCoordinate_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        reseq_interfaces__srv__ComputeCoordinate_Response__fini(&data[i - 1]);
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
reseq_interfaces__srv__ComputeCoordinate_Response__Sequence__fini(reseq_interfaces__srv__ComputeCoordinate_Response__Sequence * array)
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
      reseq_interfaces__srv__ComputeCoordinate_Response__fini(&array->data[i]);
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

reseq_interfaces__srv__ComputeCoordinate_Response__Sequence *
reseq_interfaces__srv__ComputeCoordinate_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  reseq_interfaces__srv__ComputeCoordinate_Response__Sequence * array = (reseq_interfaces__srv__ComputeCoordinate_Response__Sequence *)allocator.allocate(sizeof(reseq_interfaces__srv__ComputeCoordinate_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = reseq_interfaces__srv__ComputeCoordinate_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
reseq_interfaces__srv__ComputeCoordinate_Response__Sequence__destroy(reseq_interfaces__srv__ComputeCoordinate_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    reseq_interfaces__srv__ComputeCoordinate_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
reseq_interfaces__srv__ComputeCoordinate_Response__Sequence__are_equal(const reseq_interfaces__srv__ComputeCoordinate_Response__Sequence * lhs, const reseq_interfaces__srv__ComputeCoordinate_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!reseq_interfaces__srv__ComputeCoordinate_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
reseq_interfaces__srv__ComputeCoordinate_Response__Sequence__copy(
  const reseq_interfaces__srv__ComputeCoordinate_Response__Sequence * input,
  reseq_interfaces__srv__ComputeCoordinate_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(reseq_interfaces__srv__ComputeCoordinate_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    reseq_interfaces__srv__ComputeCoordinate_Response * data =
      (reseq_interfaces__srv__ComputeCoordinate_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!reseq_interfaces__srv__ComputeCoordinate_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          reseq_interfaces__srv__ComputeCoordinate_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!reseq_interfaces__srv__ComputeCoordinate_Response__copy(
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
// #include "reseq_interfaces/srv/detail/compute_coordinate__functions.h"

bool
reseq_interfaces__srv__ComputeCoordinate_Event__init(reseq_interfaces__srv__ComputeCoordinate_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    reseq_interfaces__srv__ComputeCoordinate_Event__fini(msg);
    return false;
  }
  // request
  if (!reseq_interfaces__srv__ComputeCoordinate_Request__Sequence__init(&msg->request, 0)) {
    reseq_interfaces__srv__ComputeCoordinate_Event__fini(msg);
    return false;
  }
  // response
  if (!reseq_interfaces__srv__ComputeCoordinate_Response__Sequence__init(&msg->response, 0)) {
    reseq_interfaces__srv__ComputeCoordinate_Event__fini(msg);
    return false;
  }
  return true;
}

void
reseq_interfaces__srv__ComputeCoordinate_Event__fini(reseq_interfaces__srv__ComputeCoordinate_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  reseq_interfaces__srv__ComputeCoordinate_Request__Sequence__fini(&msg->request);
  // response
  reseq_interfaces__srv__ComputeCoordinate_Response__Sequence__fini(&msg->response);
}

bool
reseq_interfaces__srv__ComputeCoordinate_Event__are_equal(const reseq_interfaces__srv__ComputeCoordinate_Event * lhs, const reseq_interfaces__srv__ComputeCoordinate_Event * rhs)
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
  if (!reseq_interfaces__srv__ComputeCoordinate_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!reseq_interfaces__srv__ComputeCoordinate_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
reseq_interfaces__srv__ComputeCoordinate_Event__copy(
  const reseq_interfaces__srv__ComputeCoordinate_Event * input,
  reseq_interfaces__srv__ComputeCoordinate_Event * output)
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
  if (!reseq_interfaces__srv__ComputeCoordinate_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!reseq_interfaces__srv__ComputeCoordinate_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

reseq_interfaces__srv__ComputeCoordinate_Event *
reseq_interfaces__srv__ComputeCoordinate_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  reseq_interfaces__srv__ComputeCoordinate_Event * msg = (reseq_interfaces__srv__ComputeCoordinate_Event *)allocator.allocate(sizeof(reseq_interfaces__srv__ComputeCoordinate_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(reseq_interfaces__srv__ComputeCoordinate_Event));
  bool success = reseq_interfaces__srv__ComputeCoordinate_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
reseq_interfaces__srv__ComputeCoordinate_Event__destroy(reseq_interfaces__srv__ComputeCoordinate_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    reseq_interfaces__srv__ComputeCoordinate_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
reseq_interfaces__srv__ComputeCoordinate_Event__Sequence__init(reseq_interfaces__srv__ComputeCoordinate_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  reseq_interfaces__srv__ComputeCoordinate_Event * data = NULL;

  if (size) {
    data = (reseq_interfaces__srv__ComputeCoordinate_Event *)allocator.zero_allocate(size, sizeof(reseq_interfaces__srv__ComputeCoordinate_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = reseq_interfaces__srv__ComputeCoordinate_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        reseq_interfaces__srv__ComputeCoordinate_Event__fini(&data[i - 1]);
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
reseq_interfaces__srv__ComputeCoordinate_Event__Sequence__fini(reseq_interfaces__srv__ComputeCoordinate_Event__Sequence * array)
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
      reseq_interfaces__srv__ComputeCoordinate_Event__fini(&array->data[i]);
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

reseq_interfaces__srv__ComputeCoordinate_Event__Sequence *
reseq_interfaces__srv__ComputeCoordinate_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  reseq_interfaces__srv__ComputeCoordinate_Event__Sequence * array = (reseq_interfaces__srv__ComputeCoordinate_Event__Sequence *)allocator.allocate(sizeof(reseq_interfaces__srv__ComputeCoordinate_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = reseq_interfaces__srv__ComputeCoordinate_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
reseq_interfaces__srv__ComputeCoordinate_Event__Sequence__destroy(reseq_interfaces__srv__ComputeCoordinate_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    reseq_interfaces__srv__ComputeCoordinate_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
reseq_interfaces__srv__ComputeCoordinate_Event__Sequence__are_equal(const reseq_interfaces__srv__ComputeCoordinate_Event__Sequence * lhs, const reseq_interfaces__srv__ComputeCoordinate_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!reseq_interfaces__srv__ComputeCoordinate_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
reseq_interfaces__srv__ComputeCoordinate_Event__Sequence__copy(
  const reseq_interfaces__srv__ComputeCoordinate_Event__Sequence * input,
  reseq_interfaces__srv__ComputeCoordinate_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(reseq_interfaces__srv__ComputeCoordinate_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    reseq_interfaces__srv__ComputeCoordinate_Event * data =
      (reseq_interfaces__srv__ComputeCoordinate_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!reseq_interfaces__srv__ComputeCoordinate_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          reseq_interfaces__srv__ComputeCoordinate_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!reseq_interfaces__srv__ComputeCoordinate_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
