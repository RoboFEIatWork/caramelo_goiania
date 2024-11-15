// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from grid_map_msgs:srv/ProcessFile.idl
// generated code does not contain a copyright notice
#include "grid_map_msgs/srv/detail/process_file__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `file_path`
// Member `topic_name`
#include "rosidl_runtime_c/string_functions.h"

bool
grid_map_msgs__srv__ProcessFile_Request__init(grid_map_msgs__srv__ProcessFile_Request * msg)
{
  if (!msg) {
    return false;
  }
  // file_path
  if (!rosidl_runtime_c__String__init(&msg->file_path)) {
    grid_map_msgs__srv__ProcessFile_Request__fini(msg);
    return false;
  }
  // topic_name
  if (!rosidl_runtime_c__String__init(&msg->topic_name)) {
    grid_map_msgs__srv__ProcessFile_Request__fini(msg);
    return false;
  }
  return true;
}

void
grid_map_msgs__srv__ProcessFile_Request__fini(grid_map_msgs__srv__ProcessFile_Request * msg)
{
  if (!msg) {
    return;
  }
  // file_path
  rosidl_runtime_c__String__fini(&msg->file_path);
  // topic_name
  rosidl_runtime_c__String__fini(&msg->topic_name);
}

bool
grid_map_msgs__srv__ProcessFile_Request__are_equal(const grid_map_msgs__srv__ProcessFile_Request * lhs, const grid_map_msgs__srv__ProcessFile_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // file_path
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->file_path), &(rhs->file_path)))
  {
    return false;
  }
  // topic_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->topic_name), &(rhs->topic_name)))
  {
    return false;
  }
  return true;
}

bool
grid_map_msgs__srv__ProcessFile_Request__copy(
  const grid_map_msgs__srv__ProcessFile_Request * input,
  grid_map_msgs__srv__ProcessFile_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // file_path
  if (!rosidl_runtime_c__String__copy(
      &(input->file_path), &(output->file_path)))
  {
    return false;
  }
  // topic_name
  if (!rosidl_runtime_c__String__copy(
      &(input->topic_name), &(output->topic_name)))
  {
    return false;
  }
  return true;
}

grid_map_msgs__srv__ProcessFile_Request *
grid_map_msgs__srv__ProcessFile_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  grid_map_msgs__srv__ProcessFile_Request * msg = (grid_map_msgs__srv__ProcessFile_Request *)allocator.allocate(sizeof(grid_map_msgs__srv__ProcessFile_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(grid_map_msgs__srv__ProcessFile_Request));
  bool success = grid_map_msgs__srv__ProcessFile_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
grid_map_msgs__srv__ProcessFile_Request__destroy(grid_map_msgs__srv__ProcessFile_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    grid_map_msgs__srv__ProcessFile_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
grid_map_msgs__srv__ProcessFile_Request__Sequence__init(grid_map_msgs__srv__ProcessFile_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  grid_map_msgs__srv__ProcessFile_Request * data = NULL;

  if (size) {
    data = (grid_map_msgs__srv__ProcessFile_Request *)allocator.zero_allocate(size, sizeof(grid_map_msgs__srv__ProcessFile_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = grid_map_msgs__srv__ProcessFile_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        grid_map_msgs__srv__ProcessFile_Request__fini(&data[i - 1]);
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
grid_map_msgs__srv__ProcessFile_Request__Sequence__fini(grid_map_msgs__srv__ProcessFile_Request__Sequence * array)
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
      grid_map_msgs__srv__ProcessFile_Request__fini(&array->data[i]);
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

grid_map_msgs__srv__ProcessFile_Request__Sequence *
grid_map_msgs__srv__ProcessFile_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  grid_map_msgs__srv__ProcessFile_Request__Sequence * array = (grid_map_msgs__srv__ProcessFile_Request__Sequence *)allocator.allocate(sizeof(grid_map_msgs__srv__ProcessFile_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = grid_map_msgs__srv__ProcessFile_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
grid_map_msgs__srv__ProcessFile_Request__Sequence__destroy(grid_map_msgs__srv__ProcessFile_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    grid_map_msgs__srv__ProcessFile_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
grid_map_msgs__srv__ProcessFile_Request__Sequence__are_equal(const grid_map_msgs__srv__ProcessFile_Request__Sequence * lhs, const grid_map_msgs__srv__ProcessFile_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!grid_map_msgs__srv__ProcessFile_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
grid_map_msgs__srv__ProcessFile_Request__Sequence__copy(
  const grid_map_msgs__srv__ProcessFile_Request__Sequence * input,
  grid_map_msgs__srv__ProcessFile_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(grid_map_msgs__srv__ProcessFile_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    grid_map_msgs__srv__ProcessFile_Request * data =
      (grid_map_msgs__srv__ProcessFile_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!grid_map_msgs__srv__ProcessFile_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          grid_map_msgs__srv__ProcessFile_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!grid_map_msgs__srv__ProcessFile_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
grid_map_msgs__srv__ProcessFile_Response__init(grid_map_msgs__srv__ProcessFile_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  return true;
}

void
grid_map_msgs__srv__ProcessFile_Response__fini(grid_map_msgs__srv__ProcessFile_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
}

bool
grid_map_msgs__srv__ProcessFile_Response__are_equal(const grid_map_msgs__srv__ProcessFile_Response * lhs, const grid_map_msgs__srv__ProcessFile_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  return true;
}

bool
grid_map_msgs__srv__ProcessFile_Response__copy(
  const grid_map_msgs__srv__ProcessFile_Response * input,
  grid_map_msgs__srv__ProcessFile_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  return true;
}

grid_map_msgs__srv__ProcessFile_Response *
grid_map_msgs__srv__ProcessFile_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  grid_map_msgs__srv__ProcessFile_Response * msg = (grid_map_msgs__srv__ProcessFile_Response *)allocator.allocate(sizeof(grid_map_msgs__srv__ProcessFile_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(grid_map_msgs__srv__ProcessFile_Response));
  bool success = grid_map_msgs__srv__ProcessFile_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
grid_map_msgs__srv__ProcessFile_Response__destroy(grid_map_msgs__srv__ProcessFile_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    grid_map_msgs__srv__ProcessFile_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
grid_map_msgs__srv__ProcessFile_Response__Sequence__init(grid_map_msgs__srv__ProcessFile_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  grid_map_msgs__srv__ProcessFile_Response * data = NULL;

  if (size) {
    data = (grid_map_msgs__srv__ProcessFile_Response *)allocator.zero_allocate(size, sizeof(grid_map_msgs__srv__ProcessFile_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = grid_map_msgs__srv__ProcessFile_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        grid_map_msgs__srv__ProcessFile_Response__fini(&data[i - 1]);
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
grid_map_msgs__srv__ProcessFile_Response__Sequence__fini(grid_map_msgs__srv__ProcessFile_Response__Sequence * array)
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
      grid_map_msgs__srv__ProcessFile_Response__fini(&array->data[i]);
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

grid_map_msgs__srv__ProcessFile_Response__Sequence *
grid_map_msgs__srv__ProcessFile_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  grid_map_msgs__srv__ProcessFile_Response__Sequence * array = (grid_map_msgs__srv__ProcessFile_Response__Sequence *)allocator.allocate(sizeof(grid_map_msgs__srv__ProcessFile_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = grid_map_msgs__srv__ProcessFile_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
grid_map_msgs__srv__ProcessFile_Response__Sequence__destroy(grid_map_msgs__srv__ProcessFile_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    grid_map_msgs__srv__ProcessFile_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
grid_map_msgs__srv__ProcessFile_Response__Sequence__are_equal(const grid_map_msgs__srv__ProcessFile_Response__Sequence * lhs, const grid_map_msgs__srv__ProcessFile_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!grid_map_msgs__srv__ProcessFile_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
grid_map_msgs__srv__ProcessFile_Response__Sequence__copy(
  const grid_map_msgs__srv__ProcessFile_Response__Sequence * input,
  grid_map_msgs__srv__ProcessFile_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(grid_map_msgs__srv__ProcessFile_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    grid_map_msgs__srv__ProcessFile_Response * data =
      (grid_map_msgs__srv__ProcessFile_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!grid_map_msgs__srv__ProcessFile_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          grid_map_msgs__srv__ProcessFile_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!grid_map_msgs__srv__ProcessFile_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
