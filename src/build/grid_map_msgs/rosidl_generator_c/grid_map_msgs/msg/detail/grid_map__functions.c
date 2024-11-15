// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from grid_map_msgs:msg/GridMap.idl
// generated code does not contain a copyright notice
#include "grid_map_msgs/msg/detail/grid_map__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `info`
#include "grid_map_msgs/msg/detail/grid_map_info__functions.h"
// Member `layers`
// Member `basic_layers`
#include "rosidl_runtime_c/string_functions.h"
// Member `data`
#include "std_msgs/msg/detail/float32_multi_array__functions.h"

bool
grid_map_msgs__msg__GridMap__init(grid_map_msgs__msg__GridMap * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    grid_map_msgs__msg__GridMap__fini(msg);
    return false;
  }
  // info
  if (!grid_map_msgs__msg__GridMapInfo__init(&msg->info)) {
    grid_map_msgs__msg__GridMap__fini(msg);
    return false;
  }
  // layers
  if (!rosidl_runtime_c__String__Sequence__init(&msg->layers, 0)) {
    grid_map_msgs__msg__GridMap__fini(msg);
    return false;
  }
  // basic_layers
  if (!rosidl_runtime_c__String__Sequence__init(&msg->basic_layers, 0)) {
    grid_map_msgs__msg__GridMap__fini(msg);
    return false;
  }
  // data
  if (!std_msgs__msg__Float32MultiArray__Sequence__init(&msg->data, 0)) {
    grid_map_msgs__msg__GridMap__fini(msg);
    return false;
  }
  // outer_start_index
  // inner_start_index
  return true;
}

void
grid_map_msgs__msg__GridMap__fini(grid_map_msgs__msg__GridMap * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // info
  grid_map_msgs__msg__GridMapInfo__fini(&msg->info);
  // layers
  rosidl_runtime_c__String__Sequence__fini(&msg->layers);
  // basic_layers
  rosidl_runtime_c__String__Sequence__fini(&msg->basic_layers);
  // data
  std_msgs__msg__Float32MultiArray__Sequence__fini(&msg->data);
  // outer_start_index
  // inner_start_index
}

bool
grid_map_msgs__msg__GridMap__are_equal(const grid_map_msgs__msg__GridMap * lhs, const grid_map_msgs__msg__GridMap * rhs)
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
  // info
  if (!grid_map_msgs__msg__GridMapInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // layers
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->layers), &(rhs->layers)))
  {
    return false;
  }
  // basic_layers
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->basic_layers), &(rhs->basic_layers)))
  {
    return false;
  }
  // data
  if (!std_msgs__msg__Float32MultiArray__Sequence__are_equal(
      &(lhs->data), &(rhs->data)))
  {
    return false;
  }
  // outer_start_index
  if (lhs->outer_start_index != rhs->outer_start_index) {
    return false;
  }
  // inner_start_index
  if (lhs->inner_start_index != rhs->inner_start_index) {
    return false;
  }
  return true;
}

bool
grid_map_msgs__msg__GridMap__copy(
  const grid_map_msgs__msg__GridMap * input,
  grid_map_msgs__msg__GridMap * output)
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
  // info
  if (!grid_map_msgs__msg__GridMapInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // layers
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->layers), &(output->layers)))
  {
    return false;
  }
  // basic_layers
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->basic_layers), &(output->basic_layers)))
  {
    return false;
  }
  // data
  if (!std_msgs__msg__Float32MultiArray__Sequence__copy(
      &(input->data), &(output->data)))
  {
    return false;
  }
  // outer_start_index
  output->outer_start_index = input->outer_start_index;
  // inner_start_index
  output->inner_start_index = input->inner_start_index;
  return true;
}

grid_map_msgs__msg__GridMap *
grid_map_msgs__msg__GridMap__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  grid_map_msgs__msg__GridMap * msg = (grid_map_msgs__msg__GridMap *)allocator.allocate(sizeof(grid_map_msgs__msg__GridMap), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(grid_map_msgs__msg__GridMap));
  bool success = grid_map_msgs__msg__GridMap__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
grid_map_msgs__msg__GridMap__destroy(grid_map_msgs__msg__GridMap * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    grid_map_msgs__msg__GridMap__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
grid_map_msgs__msg__GridMap__Sequence__init(grid_map_msgs__msg__GridMap__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  grid_map_msgs__msg__GridMap * data = NULL;

  if (size) {
    data = (grid_map_msgs__msg__GridMap *)allocator.zero_allocate(size, sizeof(grid_map_msgs__msg__GridMap), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = grid_map_msgs__msg__GridMap__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        grid_map_msgs__msg__GridMap__fini(&data[i - 1]);
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
grid_map_msgs__msg__GridMap__Sequence__fini(grid_map_msgs__msg__GridMap__Sequence * array)
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
      grid_map_msgs__msg__GridMap__fini(&array->data[i]);
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

grid_map_msgs__msg__GridMap__Sequence *
grid_map_msgs__msg__GridMap__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  grid_map_msgs__msg__GridMap__Sequence * array = (grid_map_msgs__msg__GridMap__Sequence *)allocator.allocate(sizeof(grid_map_msgs__msg__GridMap__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = grid_map_msgs__msg__GridMap__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
grid_map_msgs__msg__GridMap__Sequence__destroy(grid_map_msgs__msg__GridMap__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    grid_map_msgs__msg__GridMap__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
grid_map_msgs__msg__GridMap__Sequence__are_equal(const grid_map_msgs__msg__GridMap__Sequence * lhs, const grid_map_msgs__msg__GridMap__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!grid_map_msgs__msg__GridMap__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
grid_map_msgs__msg__GridMap__Sequence__copy(
  const grid_map_msgs__msg__GridMap__Sequence * input,
  grid_map_msgs__msg__GridMap__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(grid_map_msgs__msg__GridMap);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    grid_map_msgs__msg__GridMap * data =
      (grid_map_msgs__msg__GridMap *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!grid_map_msgs__msg__GridMap__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          grid_map_msgs__msg__GridMap__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!grid_map_msgs__msg__GridMap__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
