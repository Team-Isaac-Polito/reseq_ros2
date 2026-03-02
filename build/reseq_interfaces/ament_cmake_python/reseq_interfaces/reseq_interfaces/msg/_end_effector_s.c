// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from reseq_interfaces:msg/EndEffector.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "reseq_interfaces/msg/detail/end_effector__struct.h"
#include "reseq_interfaces/msg/detail/end_effector__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool reseq_interfaces__msg__end_effector__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[47];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("reseq_interfaces.msg._end_effector.EndEffector", full_classname_dest, 46) == 0);
  }
  reseq_interfaces__msg__EndEffector * ros_message = _ros_message;
  {  // pitch_vel
    PyObject * field = PyObject_GetAttrString(_pymsg, "pitch_vel");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->pitch_vel = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // head_pitch_vel
    PyObject * field = PyObject_GetAttrString(_pymsg, "head_pitch_vel");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->head_pitch_vel = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // head_roll_vel
    PyObject * field = PyObject_GetAttrString(_pymsg, "head_roll_vel");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->head_roll_vel = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * reseq_interfaces__msg__end_effector__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of EndEffector */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("reseq_interfaces.msg._end_effector");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "EndEffector");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  reseq_interfaces__msg__EndEffector * ros_message = (reseq_interfaces__msg__EndEffector *)raw_ros_message;
  {  // pitch_vel
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->pitch_vel);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pitch_vel", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // head_pitch_vel
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->head_pitch_vel);
    {
      int rc = PyObject_SetAttrString(_pymessage, "head_pitch_vel", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // head_roll_vel
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->head_roll_vel);
    {
      int rc = PyObject_SetAttrString(_pymessage, "head_roll_vel", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
