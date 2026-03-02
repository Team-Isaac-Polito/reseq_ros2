// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from reseq_interfaces:srv/BatchDetections2D.idl
// generated code does not contain a copyright notice
#include "reseq_interfaces/srv/detail/batch_detections2_d__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `robot`
// Member `mode`
#include "rosidl_runtime_c/string_functions.h"
// Member `detections`
#include "vision_msgs/msg/detail/detection2_d_array__functions.h"

bool
reseq_interfaces__srv__BatchDetections2D_Request__init(reseq_interfaces__srv__BatchDetections2D_Request * msg)
{
  if (!msg) {
    return false;
  }
  // robot
  if (!rosidl_runtime_c__String__init(&msg->robot)) {
    reseq_interfaces__srv__BatchDetections2D_Request__fini(msg);
    return false;
  }
  // mode
  if (!rosidl_runtime_c__String__init(&msg->mode)) {
    reseq_interfaces__srv__BatchDetections2D_Request__fini(msg);
    return false;
  }
  // detections
  if (!vision_msgs__msg__Detection2DArray__init(&msg->detections)) {
    reseq_interfaces__srv__BatchDetections2D_Request__fini(msg);
    return false;
  }
  return true;
}

void
reseq_interfaces__srv__BatchDetections2D_Request__fini(reseq_interfaces__srv__BatchDetections2D_Request * msg)
{
  if (!msg) {
    return;
  }
  // robot
  rosidl_runtime_c__String__fini(&msg->robot);
  // mode
  rosidl_runtime_c__String__fini(&msg->mode);
  // detections
  vision_msgs__msg__Detection2DArray__fini(&msg->detections);
}

bool
reseq_interfaces__srv__BatchDetections2D_Request__are_equal(const reseq_interfaces__srv__BatchDetections2D_Request * lhs, const reseq_interfaces__srv__BatchDetections2D_Request * rhs)
{
  if (!lhs || !rhs) {
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
  // detections
  if (!vision_msgs__msg__Detection2DArray__are_equal(
      &(lhs->detections), &(rhs->detections)))
  {
    return false;
  }
  return true;
}

bool
reseq_interfaces__srv__BatchDetections2D_Request__copy(
  const reseq_interfaces__srv__BatchDetections2D_Request * input,
  reseq_interfaces__srv__BatchDetections2D_Request * output)
{
  if (!input || !output) {
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
  // detections
  if (!vision_msgs__msg__Detection2DArray__copy(
      &(input->detections), &(output->detections)))
  {
    return false;
  }
  return true;
}

reseq_interfaces__srv__BatchDetections2D_Request *
reseq_interfaces__srv__BatchDetections2D_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  reseq_interfaces__srv__BatchDetections2D_Request * msg = (reseq_interfaces__srv__BatchDetections2D_Request *)allocator.allocate(sizeof(reseq_interfaces__srv__BatchDetections2D_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(reseq_interfaces__srv__BatchDetections2D_Request));
  bool success = reseq_interfaces__srv__BatchDetections2D_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
reseq_interfaces__srv__BatchDetections2D_Request__destroy(reseq_interfaces__srv__BatchDetections2D_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    reseq_interfaces__srv__BatchDetections2D_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
reseq_interfaces__srv__BatchDetections2D_Request__Sequence__init(reseq_interfaces__srv__BatchDetections2D_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  reseq_interfaces__srv__BatchDetections2D_Request * data = NULL;

  if (size) {
    data = (reseq_interfaces__srv__BatchDetections2D_Request *)allocator.zero_allocate(size, sizeof(reseq_interfaces__srv__BatchDetections2D_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = reseq_interfaces__srv__BatchDetections2D_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        reseq_interfaces__srv__BatchDetections2D_Request__fini(&data[i - 1]);
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
reseq_interfaces__srv__BatchDetections2D_Request__Sequence__fini(reseq_interfaces__srv__BatchDetections2D_Request__Sequence * array)
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
      reseq_interfaces__srv__BatchDetections2D_Request__fini(&array->data[i]);
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

reseq_interfaces__srv__BatchDetections2D_Request__Sequence *
reseq_interfaces__srv__BatchDetections2D_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  reseq_interfaces__srv__BatchDetections2D_Request__Sequence * array = (reseq_interfaces__srv__BatchDetections2D_Request__Sequence *)allocator.allocate(sizeof(reseq_interfaces__srv__BatchDetections2D_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = reseq_interfaces__srv__BatchDetections2D_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
reseq_interfaces__srv__BatchDetections2D_Request__Sequence__destroy(reseq_interfaces__srv__BatchDetections2D_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    reseq_interfaces__srv__BatchDetections2D_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
reseq_interfaces__srv__BatchDetections2D_Request__Sequence__are_equal(const reseq_interfaces__srv__BatchDetections2D_Request__Sequence * lhs, const reseq_interfaces__srv__BatchDetections2D_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!reseq_interfaces__srv__BatchDetections2D_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
reseq_interfaces__srv__BatchDetections2D_Request__Sequence__copy(
  const reseq_interfaces__srv__BatchDetections2D_Request__Sequence * input,
  reseq_interfaces__srv__BatchDetections2D_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(reseq_interfaces__srv__BatchDetections2D_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    reseq_interfaces__srv__BatchDetections2D_Request * data =
      (reseq_interfaces__srv__BatchDetections2D_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!reseq_interfaces__srv__BatchDetections2D_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          reseq_interfaces__srv__BatchDetections2D_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!reseq_interfaces__srv__BatchDetections2D_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
reseq_interfaces__srv__BatchDetections2D_Response__init(reseq_interfaces__srv__BatchDetections2D_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    reseq_interfaces__srv__BatchDetections2D_Response__fini(msg);
    return false;
  }
  return true;
}

void
reseq_interfaces__srv__BatchDetections2D_Response__fini(reseq_interfaces__srv__BatchDetections2D_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
reseq_interfaces__srv__BatchDetections2D_Response__are_equal(const reseq_interfaces__srv__BatchDetections2D_Response * lhs, const reseq_interfaces__srv__BatchDetections2D_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
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
reseq_interfaces__srv__BatchDetections2D_Response__copy(
  const reseq_interfaces__srv__BatchDetections2D_Response * input,
  reseq_interfaces__srv__BatchDetections2D_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  return true;
}

reseq_interfaces__srv__BatchDetections2D_Response *
reseq_interfaces__srv__BatchDetections2D_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  reseq_interfaces__srv__BatchDetections2D_Response * msg = (reseq_interfaces__srv__BatchDetections2D_Response *)allocator.allocate(sizeof(reseq_interfaces__srv__BatchDetections2D_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(reseq_interfaces__srv__BatchDetections2D_Response));
  bool success = reseq_interfaces__srv__BatchDetections2D_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
reseq_interfaces__srv__BatchDetections2D_Response__destroy(reseq_interfaces__srv__BatchDetections2D_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    reseq_interfaces__srv__BatchDetections2D_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
reseq_interfaces__srv__BatchDetections2D_Response__Sequence__init(reseq_interfaces__srv__BatchDetections2D_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  reseq_interfaces__srv__BatchDetections2D_Response * data = NULL;

  if (size) {
    data = (reseq_interfaces__srv__BatchDetections2D_Response *)allocator.zero_allocate(size, sizeof(reseq_interfaces__srv__BatchDetections2D_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = reseq_interfaces__srv__BatchDetections2D_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        reseq_interfaces__srv__BatchDetections2D_Response__fini(&data[i - 1]);
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
reseq_interfaces__srv__BatchDetections2D_Response__Sequence__fini(reseq_interfaces__srv__BatchDetections2D_Response__Sequence * array)
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
      reseq_interfaces__srv__BatchDetections2D_Response__fini(&array->data[i]);
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

reseq_interfaces__srv__BatchDetections2D_Response__Sequence *
reseq_interfaces__srv__BatchDetections2D_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  reseq_interfaces__srv__BatchDetections2D_Response__Sequence * array = (reseq_interfaces__srv__BatchDetections2D_Response__Sequence *)allocator.allocate(sizeof(reseq_interfaces__srv__BatchDetections2D_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = reseq_interfaces__srv__BatchDetections2D_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
reseq_interfaces__srv__BatchDetections2D_Response__Sequence__destroy(reseq_interfaces__srv__BatchDetections2D_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    reseq_interfaces__srv__BatchDetections2D_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
reseq_interfaces__srv__BatchDetections2D_Response__Sequence__are_equal(const reseq_interfaces__srv__BatchDetections2D_Response__Sequence * lhs, const reseq_interfaces__srv__BatchDetections2D_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!reseq_interfaces__srv__BatchDetections2D_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
reseq_interfaces__srv__BatchDetections2D_Response__Sequence__copy(
  const reseq_interfaces__srv__BatchDetections2D_Response__Sequence * input,
  reseq_interfaces__srv__BatchDetections2D_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(reseq_interfaces__srv__BatchDetections2D_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    reseq_interfaces__srv__BatchDetections2D_Response * data =
      (reseq_interfaces__srv__BatchDetections2D_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!reseq_interfaces__srv__BatchDetections2D_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          reseq_interfaces__srv__BatchDetections2D_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!reseq_interfaces__srv__BatchDetections2D_Response__copy(
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
// #include "reseq_interfaces/srv/detail/batch_detections2_d__functions.h"

bool
reseq_interfaces__srv__BatchDetections2D_Event__init(reseq_interfaces__srv__BatchDetections2D_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    reseq_interfaces__srv__BatchDetections2D_Event__fini(msg);
    return false;
  }
  // request
  if (!reseq_interfaces__srv__BatchDetections2D_Request__Sequence__init(&msg->request, 0)) {
    reseq_interfaces__srv__BatchDetections2D_Event__fini(msg);
    return false;
  }
  // response
  if (!reseq_interfaces__srv__BatchDetections2D_Response__Sequence__init(&msg->response, 0)) {
    reseq_interfaces__srv__BatchDetections2D_Event__fini(msg);
    return false;
  }
  return true;
}

void
reseq_interfaces__srv__BatchDetections2D_Event__fini(reseq_interfaces__srv__BatchDetections2D_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  reseq_interfaces__srv__BatchDetections2D_Request__Sequence__fini(&msg->request);
  // response
  reseq_interfaces__srv__BatchDetections2D_Response__Sequence__fini(&msg->response);
}

bool
reseq_interfaces__srv__BatchDetections2D_Event__are_equal(const reseq_interfaces__srv__BatchDetections2D_Event * lhs, const reseq_interfaces__srv__BatchDetections2D_Event * rhs)
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
  if (!reseq_interfaces__srv__BatchDetections2D_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!reseq_interfaces__srv__BatchDetections2D_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
reseq_interfaces__srv__BatchDetections2D_Event__copy(
  const reseq_interfaces__srv__BatchDetections2D_Event * input,
  reseq_interfaces__srv__BatchDetections2D_Event * output)
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
  if (!reseq_interfaces__srv__BatchDetections2D_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!reseq_interfaces__srv__BatchDetections2D_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

reseq_interfaces__srv__BatchDetections2D_Event *
reseq_interfaces__srv__BatchDetections2D_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  reseq_interfaces__srv__BatchDetections2D_Event * msg = (reseq_interfaces__srv__BatchDetections2D_Event *)allocator.allocate(sizeof(reseq_interfaces__srv__BatchDetections2D_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(reseq_interfaces__srv__BatchDetections2D_Event));
  bool success = reseq_interfaces__srv__BatchDetections2D_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
reseq_interfaces__srv__BatchDetections2D_Event__destroy(reseq_interfaces__srv__BatchDetections2D_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    reseq_interfaces__srv__BatchDetections2D_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
reseq_interfaces__srv__BatchDetections2D_Event__Sequence__init(reseq_interfaces__srv__BatchDetections2D_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  reseq_interfaces__srv__BatchDetections2D_Event * data = NULL;

  if (size) {
    data = (reseq_interfaces__srv__BatchDetections2D_Event *)allocator.zero_allocate(size, sizeof(reseq_interfaces__srv__BatchDetections2D_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = reseq_interfaces__srv__BatchDetections2D_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        reseq_interfaces__srv__BatchDetections2D_Event__fini(&data[i - 1]);
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
reseq_interfaces__srv__BatchDetections2D_Event__Sequence__fini(reseq_interfaces__srv__BatchDetections2D_Event__Sequence * array)
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
      reseq_interfaces__srv__BatchDetections2D_Event__fini(&array->data[i]);
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

reseq_interfaces__srv__BatchDetections2D_Event__Sequence *
reseq_interfaces__srv__BatchDetections2D_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  reseq_interfaces__srv__BatchDetections2D_Event__Sequence * array = (reseq_interfaces__srv__BatchDetections2D_Event__Sequence *)allocator.allocate(sizeof(reseq_interfaces__srv__BatchDetections2D_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = reseq_interfaces__srv__BatchDetections2D_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
reseq_interfaces__srv__BatchDetections2D_Event__Sequence__destroy(reseq_interfaces__srv__BatchDetections2D_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    reseq_interfaces__srv__BatchDetections2D_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
reseq_interfaces__srv__BatchDetections2D_Event__Sequence__are_equal(const reseq_interfaces__srv__BatchDetections2D_Event__Sequence * lhs, const reseq_interfaces__srv__BatchDetections2D_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!reseq_interfaces__srv__BatchDetections2D_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
reseq_interfaces__srv__BatchDetections2D_Event__Sequence__copy(
  const reseq_interfaces__srv__BatchDetections2D_Event__Sequence * input,
  reseq_interfaces__srv__BatchDetections2D_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(reseq_interfaces__srv__BatchDetections2D_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    reseq_interfaces__srv__BatchDetections2D_Event * data =
      (reseq_interfaces__srv__BatchDetections2D_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!reseq_interfaces__srv__BatchDetections2D_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          reseq_interfaces__srv__BatchDetections2D_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!reseq_interfaces__srv__BatchDetections2D_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
