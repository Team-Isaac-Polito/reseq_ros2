# generated from rosidl_generator_py/resource/_idl.py.em
# with input from reseq_interfaces:srv/GetStatus.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GetStatus_Request(type):
    """Metaclass of message 'GetStatus_Request'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('reseq_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'reseq_interfaces.srv.GetStatus_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_status__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_status__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_status__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_status__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_status__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetStatus_Request(metaclass=Metaclass_GetStatus_Request):
    """Message class 'GetStatus_Request'."""

    __slots__ = [
        '_check_fields',
    ]

    _fields_and_field_types = {
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
    )

    def __init__(self, **kwargs):
        if 'check_fields' in kwargs:
            self._check_fields = kwargs['check_fields']
        else:
            self._check_fields = ros_python_check_fields == '1'
        if self._check_fields:
            assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
                'Invalid arguments passed to constructor: %s' % \
                ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.get_fields_and_field_types().keys(), self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    if self._check_fields:
                        assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)


# Import statements for member types

import builtins  # noqa: E402, I100

# already imported above
# import rosidl_parser.definition


class Metaclass_GetStatus_Response(type):
    """Metaclass of message 'GetStatus_Response'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('reseq_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'reseq_interfaces.srv.GetStatus_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_status__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_status__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_status__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_status__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_status__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetStatus_Response(metaclass=Metaclass_GetStatus_Response):
    """Message class 'GetStatus_Response'."""

    __slots__ = [
        '_current_mode',
        '_initialized',
        '_csv_path',
        '_last_error',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'current_mode': 'uint8',
        'initialized': 'boolean',
        'csv_path': 'string',
        'last_error': 'string',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        if 'check_fields' in kwargs:
            self._check_fields = kwargs['check_fields']
        else:
            self._check_fields = ros_python_check_fields == '1'
        if self._check_fields:
            assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
                'Invalid arguments passed to constructor: %s' % \
                ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.current_mode = kwargs.get('current_mode', int())
        self.initialized = kwargs.get('initialized', bool())
        self.csv_path = kwargs.get('csv_path', str())
        self.last_error = kwargs.get('last_error', str())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.get_fields_and_field_types().keys(), self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    if self._check_fields:
                        assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.current_mode != other.current_mode:
            return False
        if self.initialized != other.initialized:
            return False
        if self.csv_path != other.csv_path:
            return False
        if self.last_error != other.last_error:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def current_mode(self):
        """Message field 'current_mode'."""
        return self._current_mode

    @current_mode.setter
    def current_mode(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'current_mode' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'current_mode' field must be an unsigned integer in [0, 255]"
        self._current_mode = value

    @builtins.property
    def initialized(self):
        """Message field 'initialized'."""
        return self._initialized

    @initialized.setter
    def initialized(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'initialized' field must be of type 'bool'"
        self._initialized = value

    @builtins.property
    def csv_path(self):
        """Message field 'csv_path'."""
        return self._csv_path

    @csv_path.setter
    def csv_path(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'csv_path' field must be of type 'str'"
        self._csv_path = value

    @builtins.property
    def last_error(self):
        """Message field 'last_error'."""
        return self._last_error

    @last_error.setter
    def last_error(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'last_error' field must be of type 'str'"
        self._last_error = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_GetStatus_Event(type):
    """Metaclass of message 'GetStatus_Event'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('reseq_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'reseq_interfaces.srv.GetStatus_Event')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_status__event
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_status__event
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_status__event
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_status__event
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_status__event

            from service_msgs.msg import ServiceEventInfo
            if ServiceEventInfo.__class__._TYPE_SUPPORT is None:
                ServiceEventInfo.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetStatus_Event(metaclass=Metaclass_GetStatus_Event):
    """Message class 'GetStatus_Event'."""

    __slots__ = [
        '_info',
        '_request',
        '_response',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'info': 'service_msgs/ServiceEventInfo',
        'request': 'sequence<reseq_interfaces/GetStatus_Request, 1>',
        'response': 'sequence<reseq_interfaces/GetStatus_Response, 1>',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['service_msgs', 'msg'], 'ServiceEventInfo'),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.NamespacedType(['reseq_interfaces', 'srv'], 'GetStatus_Request'), 1),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.NamespacedType(['reseq_interfaces', 'srv'], 'GetStatus_Response'), 1),  # noqa: E501
    )

    def __init__(self, **kwargs):
        if 'check_fields' in kwargs:
            self._check_fields = kwargs['check_fields']
        else:
            self._check_fields = ros_python_check_fields == '1'
        if self._check_fields:
            assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
                'Invalid arguments passed to constructor: %s' % \
                ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from service_msgs.msg import ServiceEventInfo
        self.info = kwargs.get('info', ServiceEventInfo())
        self.request = kwargs.get('request', [])
        self.response = kwargs.get('response', [])

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.get_fields_and_field_types().keys(), self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    if self._check_fields:
                        assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.info != other.info:
            return False
        if self.request != other.request:
            return False
        if self.response != other.response:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def info(self):
        """Message field 'info'."""
        return self._info

    @info.setter
    def info(self, value):
        if self._check_fields:
            from service_msgs.msg import ServiceEventInfo
            assert \
                isinstance(value, ServiceEventInfo), \
                "The 'info' field must be a sub message of type 'ServiceEventInfo'"
        self._info = value

    @builtins.property
    def request(self):
        """Message field 'request'."""
        return self._request

    @request.setter
    def request(self, value):
        if self._check_fields:
            from reseq_interfaces.srv import GetStatus_Request
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) <= 1 and
                 all(isinstance(v, GetStatus_Request) for v in value) and
                 True), \
                "The 'request' field must be a set or sequence with length <= 1 and each value of type 'GetStatus_Request'"
        self._request = value

    @builtins.property
    def response(self):
        """Message field 'response'."""
        return self._response

    @response.setter
    def response(self, value):
        if self._check_fields:
            from reseq_interfaces.srv import GetStatus_Response
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) <= 1 and
                 all(isinstance(v, GetStatus_Response) for v in value) and
                 True), \
                "The 'response' field must be a set or sequence with length <= 1 and each value of type 'GetStatus_Response'"
        self._response = value


class Metaclass_GetStatus(type):
    """Metaclass of service 'GetStatus'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('reseq_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'reseq_interfaces.srv.GetStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__get_status

            from reseq_interfaces.srv import _get_status
            if _get_status.Metaclass_GetStatus_Request._TYPE_SUPPORT is None:
                _get_status.Metaclass_GetStatus_Request.__import_type_support__()
            if _get_status.Metaclass_GetStatus_Response._TYPE_SUPPORT is None:
                _get_status.Metaclass_GetStatus_Response.__import_type_support__()
            if _get_status.Metaclass_GetStatus_Event._TYPE_SUPPORT is None:
                _get_status.Metaclass_GetStatus_Event.__import_type_support__()


class GetStatus(metaclass=Metaclass_GetStatus):
    from reseq_interfaces.srv._get_status import GetStatus_Request as Request
    from reseq_interfaces.srv._get_status import GetStatus_Response as Response
    from reseq_interfaces.srv._get_status import GetStatus_Event as Event

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
