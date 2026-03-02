// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from reseq_interfaces:srv/BatchDetections2D.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "reseq_interfaces/srv/batch_detections2_d.h"


#ifndef RESEQ_INTERFACES__SRV__DETAIL__BATCH_DETECTIONS2_D__FUNCTIONS_H_
#define RESEQ_INTERFACES__SRV__DETAIL__BATCH_DETECTIONS2_D__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_runtime_c/visibility_control.h"
#include "reseq_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "reseq_interfaces/srv/detail/batch_detections2_d__struct.h"

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
const rosidl_type_hash_t *
reseq_interfaces__srv__BatchDetections2D__get_type_hash(
  const rosidl_service_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
const rosidl_runtime_c__type_description__TypeDescription *
reseq_interfaces__srv__BatchDetections2D__get_type_description(
  const rosidl_service_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
const rosidl_runtime_c__type_description__TypeSource *
reseq_interfaces__srv__BatchDetections2D__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
const rosidl_runtime_c__type_description__TypeSource__Sequence *
reseq_interfaces__srv__BatchDetections2D__get_type_description_sources(
  const rosidl_service_type_support_t * type_support);

/// Initialize srv/BatchDetections2D message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * reseq_interfaces__srv__BatchDetections2D_Request
 * )) before or use
 * reseq_interfaces__srv__BatchDetections2D_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
bool
reseq_interfaces__srv__BatchDetections2D_Request__init(reseq_interfaces__srv__BatchDetections2D_Request * msg);

/// Finalize srv/BatchDetections2D message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
void
reseq_interfaces__srv__BatchDetections2D_Request__fini(reseq_interfaces__srv__BatchDetections2D_Request * msg);

/// Create srv/BatchDetections2D message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * reseq_interfaces__srv__BatchDetections2D_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
reseq_interfaces__srv__BatchDetections2D_Request *
reseq_interfaces__srv__BatchDetections2D_Request__create(void);

/// Destroy srv/BatchDetections2D message.
/**
 * It calls
 * reseq_interfaces__srv__BatchDetections2D_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
void
reseq_interfaces__srv__BatchDetections2D_Request__destroy(reseq_interfaces__srv__BatchDetections2D_Request * msg);

/// Check for srv/BatchDetections2D message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
bool
reseq_interfaces__srv__BatchDetections2D_Request__are_equal(const reseq_interfaces__srv__BatchDetections2D_Request * lhs, const reseq_interfaces__srv__BatchDetections2D_Request * rhs);

/// Copy a srv/BatchDetections2D message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
bool
reseq_interfaces__srv__BatchDetections2D_Request__copy(
  const reseq_interfaces__srv__BatchDetections2D_Request * input,
  reseq_interfaces__srv__BatchDetections2D_Request * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
const rosidl_type_hash_t *
reseq_interfaces__srv__BatchDetections2D_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
const rosidl_runtime_c__type_description__TypeDescription *
reseq_interfaces__srv__BatchDetections2D_Request__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
const rosidl_runtime_c__type_description__TypeSource *
reseq_interfaces__srv__BatchDetections2D_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
const rosidl_runtime_c__type_description__TypeSource__Sequence *
reseq_interfaces__srv__BatchDetections2D_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of srv/BatchDetections2D messages.
/**
 * It allocates the memory for the number of elements and calls
 * reseq_interfaces__srv__BatchDetections2D_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
bool
reseq_interfaces__srv__BatchDetections2D_Request__Sequence__init(reseq_interfaces__srv__BatchDetections2D_Request__Sequence * array, size_t size);

/// Finalize array of srv/BatchDetections2D messages.
/**
 * It calls
 * reseq_interfaces__srv__BatchDetections2D_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
void
reseq_interfaces__srv__BatchDetections2D_Request__Sequence__fini(reseq_interfaces__srv__BatchDetections2D_Request__Sequence * array);

/// Create array of srv/BatchDetections2D messages.
/**
 * It allocates the memory for the array and calls
 * reseq_interfaces__srv__BatchDetections2D_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
reseq_interfaces__srv__BatchDetections2D_Request__Sequence *
reseq_interfaces__srv__BatchDetections2D_Request__Sequence__create(size_t size);

/// Destroy array of srv/BatchDetections2D messages.
/**
 * It calls
 * reseq_interfaces__srv__BatchDetections2D_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
void
reseq_interfaces__srv__BatchDetections2D_Request__Sequence__destroy(reseq_interfaces__srv__BatchDetections2D_Request__Sequence * array);

/// Check for srv/BatchDetections2D message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
bool
reseq_interfaces__srv__BatchDetections2D_Request__Sequence__are_equal(const reseq_interfaces__srv__BatchDetections2D_Request__Sequence * lhs, const reseq_interfaces__srv__BatchDetections2D_Request__Sequence * rhs);

/// Copy an array of srv/BatchDetections2D messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
bool
reseq_interfaces__srv__BatchDetections2D_Request__Sequence__copy(
  const reseq_interfaces__srv__BatchDetections2D_Request__Sequence * input,
  reseq_interfaces__srv__BatchDetections2D_Request__Sequence * output);

/// Initialize srv/BatchDetections2D message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * reseq_interfaces__srv__BatchDetections2D_Response
 * )) before or use
 * reseq_interfaces__srv__BatchDetections2D_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
bool
reseq_interfaces__srv__BatchDetections2D_Response__init(reseq_interfaces__srv__BatchDetections2D_Response * msg);

/// Finalize srv/BatchDetections2D message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
void
reseq_interfaces__srv__BatchDetections2D_Response__fini(reseq_interfaces__srv__BatchDetections2D_Response * msg);

/// Create srv/BatchDetections2D message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * reseq_interfaces__srv__BatchDetections2D_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
reseq_interfaces__srv__BatchDetections2D_Response *
reseq_interfaces__srv__BatchDetections2D_Response__create(void);

/// Destroy srv/BatchDetections2D message.
/**
 * It calls
 * reseq_interfaces__srv__BatchDetections2D_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
void
reseq_interfaces__srv__BatchDetections2D_Response__destroy(reseq_interfaces__srv__BatchDetections2D_Response * msg);

/// Check for srv/BatchDetections2D message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
bool
reseq_interfaces__srv__BatchDetections2D_Response__are_equal(const reseq_interfaces__srv__BatchDetections2D_Response * lhs, const reseq_interfaces__srv__BatchDetections2D_Response * rhs);

/// Copy a srv/BatchDetections2D message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
bool
reseq_interfaces__srv__BatchDetections2D_Response__copy(
  const reseq_interfaces__srv__BatchDetections2D_Response * input,
  reseq_interfaces__srv__BatchDetections2D_Response * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
const rosidl_type_hash_t *
reseq_interfaces__srv__BatchDetections2D_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
const rosidl_runtime_c__type_description__TypeDescription *
reseq_interfaces__srv__BatchDetections2D_Response__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
const rosidl_runtime_c__type_description__TypeSource *
reseq_interfaces__srv__BatchDetections2D_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
const rosidl_runtime_c__type_description__TypeSource__Sequence *
reseq_interfaces__srv__BatchDetections2D_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of srv/BatchDetections2D messages.
/**
 * It allocates the memory for the number of elements and calls
 * reseq_interfaces__srv__BatchDetections2D_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
bool
reseq_interfaces__srv__BatchDetections2D_Response__Sequence__init(reseq_interfaces__srv__BatchDetections2D_Response__Sequence * array, size_t size);

/// Finalize array of srv/BatchDetections2D messages.
/**
 * It calls
 * reseq_interfaces__srv__BatchDetections2D_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
void
reseq_interfaces__srv__BatchDetections2D_Response__Sequence__fini(reseq_interfaces__srv__BatchDetections2D_Response__Sequence * array);

/// Create array of srv/BatchDetections2D messages.
/**
 * It allocates the memory for the array and calls
 * reseq_interfaces__srv__BatchDetections2D_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
reseq_interfaces__srv__BatchDetections2D_Response__Sequence *
reseq_interfaces__srv__BatchDetections2D_Response__Sequence__create(size_t size);

/// Destroy array of srv/BatchDetections2D messages.
/**
 * It calls
 * reseq_interfaces__srv__BatchDetections2D_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
void
reseq_interfaces__srv__BatchDetections2D_Response__Sequence__destroy(reseq_interfaces__srv__BatchDetections2D_Response__Sequence * array);

/// Check for srv/BatchDetections2D message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
bool
reseq_interfaces__srv__BatchDetections2D_Response__Sequence__are_equal(const reseq_interfaces__srv__BatchDetections2D_Response__Sequence * lhs, const reseq_interfaces__srv__BatchDetections2D_Response__Sequence * rhs);

/// Copy an array of srv/BatchDetections2D messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
bool
reseq_interfaces__srv__BatchDetections2D_Response__Sequence__copy(
  const reseq_interfaces__srv__BatchDetections2D_Response__Sequence * input,
  reseq_interfaces__srv__BatchDetections2D_Response__Sequence * output);

/// Initialize srv/BatchDetections2D message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * reseq_interfaces__srv__BatchDetections2D_Event
 * )) before or use
 * reseq_interfaces__srv__BatchDetections2D_Event__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
bool
reseq_interfaces__srv__BatchDetections2D_Event__init(reseq_interfaces__srv__BatchDetections2D_Event * msg);

/// Finalize srv/BatchDetections2D message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
void
reseq_interfaces__srv__BatchDetections2D_Event__fini(reseq_interfaces__srv__BatchDetections2D_Event * msg);

/// Create srv/BatchDetections2D message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * reseq_interfaces__srv__BatchDetections2D_Event__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
reseq_interfaces__srv__BatchDetections2D_Event *
reseq_interfaces__srv__BatchDetections2D_Event__create(void);

/// Destroy srv/BatchDetections2D message.
/**
 * It calls
 * reseq_interfaces__srv__BatchDetections2D_Event__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
void
reseq_interfaces__srv__BatchDetections2D_Event__destroy(reseq_interfaces__srv__BatchDetections2D_Event * msg);

/// Check for srv/BatchDetections2D message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
bool
reseq_interfaces__srv__BatchDetections2D_Event__are_equal(const reseq_interfaces__srv__BatchDetections2D_Event * lhs, const reseq_interfaces__srv__BatchDetections2D_Event * rhs);

/// Copy a srv/BatchDetections2D message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
bool
reseq_interfaces__srv__BatchDetections2D_Event__copy(
  const reseq_interfaces__srv__BatchDetections2D_Event * input,
  reseq_interfaces__srv__BatchDetections2D_Event * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
const rosidl_type_hash_t *
reseq_interfaces__srv__BatchDetections2D_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
const rosidl_runtime_c__type_description__TypeDescription *
reseq_interfaces__srv__BatchDetections2D_Event__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
const rosidl_runtime_c__type_description__TypeSource *
reseq_interfaces__srv__BatchDetections2D_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
const rosidl_runtime_c__type_description__TypeSource__Sequence *
reseq_interfaces__srv__BatchDetections2D_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of srv/BatchDetections2D messages.
/**
 * It allocates the memory for the number of elements and calls
 * reseq_interfaces__srv__BatchDetections2D_Event__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
bool
reseq_interfaces__srv__BatchDetections2D_Event__Sequence__init(reseq_interfaces__srv__BatchDetections2D_Event__Sequence * array, size_t size);

/// Finalize array of srv/BatchDetections2D messages.
/**
 * It calls
 * reseq_interfaces__srv__BatchDetections2D_Event__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
void
reseq_interfaces__srv__BatchDetections2D_Event__Sequence__fini(reseq_interfaces__srv__BatchDetections2D_Event__Sequence * array);

/// Create array of srv/BatchDetections2D messages.
/**
 * It allocates the memory for the array and calls
 * reseq_interfaces__srv__BatchDetections2D_Event__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
reseq_interfaces__srv__BatchDetections2D_Event__Sequence *
reseq_interfaces__srv__BatchDetections2D_Event__Sequence__create(size_t size);

/// Destroy array of srv/BatchDetections2D messages.
/**
 * It calls
 * reseq_interfaces__srv__BatchDetections2D_Event__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
void
reseq_interfaces__srv__BatchDetections2D_Event__Sequence__destroy(reseq_interfaces__srv__BatchDetections2D_Event__Sequence * array);

/// Check for srv/BatchDetections2D message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
bool
reseq_interfaces__srv__BatchDetections2D_Event__Sequence__are_equal(const reseq_interfaces__srv__BatchDetections2D_Event__Sequence * lhs, const reseq_interfaces__srv__BatchDetections2D_Event__Sequence * rhs);

/// Copy an array of srv/BatchDetections2D messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_reseq_interfaces
bool
reseq_interfaces__srv__BatchDetections2D_Event__Sequence__copy(
  const reseq_interfaces__srv__BatchDetections2D_Event__Sequence * input,
  reseq_interfaces__srv__BatchDetections2D_Event__Sequence * output);
#ifdef __cplusplus
}
#endif

#endif  // RESEQ_INTERFACES__SRV__DETAIL__BATCH_DETECTIONS2_D__FUNCTIONS_H_
