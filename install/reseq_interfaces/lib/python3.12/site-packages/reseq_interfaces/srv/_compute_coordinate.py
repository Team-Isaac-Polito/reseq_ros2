# generated from rosidl_generator_py/resource/_idl.py.em
# with input from reseq_interfaces:srv/ComputeCoordinate.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ComputeCoordinate_Request(type):
    """Metaclass of message 'ComputeCoordinate_Request'."""

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
                'reseq_interfaces.srv.ComputeCoordinate_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__compute_coordinate__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__compute_coordinate__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__compute_coordinate__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__compute_coordinate__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__compute_coordinate__request

            from reseq_interfaces.msg import Detection
            if Detection.__class__._TYPE_SUPPORT is None:
                Detection.__class__.__import_type_support__()

            from sensor_msgs.msg import CameraInfo
            if CameraInfo.__class__._TYPE_SUPPORT is None:
                CameraInfo.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ComputeCoordinate_Request(metaclass=Metaclass_ComputeCoordinate_Request):
    """Message class 'ComputeCoordinate_Request'."""

    __slots__ = [
        '_detection',
        '_camera_info',
        '_target_frame',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'detection': 'reseq_interfaces/Detection',
        'camera_info': 'sensor_msgs/CameraInfo',
        'target_frame': 'string',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['reseq_interfaces', 'msg'], 'Detection'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['sensor_msgs', 'msg'], 'CameraInfo'),  # noqa: E501
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
        from reseq_interfaces.msg import Detection
        self.detection = kwargs.get('detection', Detection())
        from sensor_msgs.msg import CameraInfo
        self.camera_info = kwargs.get('camera_info', CameraInfo())
        self.target_frame = kwargs.get('target_frame', str())

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
        if self.detection != other.detection:
            return False
        if self.camera_info != other.camera_info:
            return False
        if self.target_frame != other.target_frame:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def detection(self):
        """Message field 'detection'."""
        return self._detection

    @detection.setter
    def detection(self, value):
        if self._check_fields:
            from reseq_interfaces.msg import Detection
            assert \
                isinstance(value, Detection), \
                "The 'detection' field must be a sub message of type 'Detection'"
        self._detection = value

    @builtins.property
    def camera_info(self):
        """Message field 'camera_info'."""
        return self._camera_info

    @camera_info.setter
    def camera_info(self, value):
        if self._check_fields:
            from sensor_msgs.msg import CameraInfo
            assert \
                isinstance(value, CameraInfo), \
                "The 'camera_info' field must be a sub message of type 'CameraInfo'"
        self._camera_info = value

    @builtins.property
    def target_frame(self):
        """Message field 'target_frame'."""
        return self._target_frame

    @target_frame.setter
    def target_frame(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'target_frame' field must be of type 'str'"
        self._target_frame = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_ComputeCoordinate_Response(type):
    """Metaclass of message 'ComputeCoordinate_Response'."""

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
                'reseq_interfaces.srv.ComputeCoordinate_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__compute_coordinate__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__compute_coordinate__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__compute_coordinate__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__compute_coordinate__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__compute_coordinate__response

            from geometry_msgs.msg import PointStamped
            if PointStamped.__class__._TYPE_SUPPORT is None:
                PointStamped.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ComputeCoordinate_Response(metaclass=Metaclass_ComputeCoordinate_Response):
    """Message class 'ComputeCoordinate_Response'."""

    __slots__ = [
        '_success',
        '_point',
        '_message',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'point': 'geometry_msgs/PointStamped',
        'message': 'string',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'PointStamped'),  # noqa: E501
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
        self.success = kwargs.get('success', bool())
        from geometry_msgs.msg import PointStamped
        self.point = kwargs.get('point', PointStamped())
        self.message = kwargs.get('message', str())

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
        if self.success != other.success:
            return False
        if self.point != other.point:
            return False
        if self.message != other.message:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def success(self):
        """Message field 'success'."""
        return self._success

    @success.setter
    def success(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'success' field must be of type 'bool'"
        self._success = value

    @builtins.property
    def point(self):
        """Message field 'point'."""
        return self._point

    @point.setter
    def point(self, value):
        if self._check_fields:
            from geometry_msgs.msg import PointStamped
            assert \
                isinstance(value, PointStamped), \
                "The 'point' field must be a sub message of type 'PointStamped'"
        self._point = value

    @builtins.property
    def message(self):
        """Message field 'message'."""
        return self._message

    @message.setter
    def message(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'message' field must be of type 'str'"
        self._message = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_ComputeCoordinate_Event(type):
    """Metaclass of message 'ComputeCoordinate_Event'."""

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
                'reseq_interfaces.srv.ComputeCoordinate_Event')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__compute_coordinate__event
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__compute_coordinate__event
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__compute_coordinate__event
            cls._TYPE_SUPPORT = module.type_support_msg__srv__compute_coordinate__event
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__compute_coordinate__event

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


class ComputeCoordinate_Event(metaclass=Metaclass_ComputeCoordinate_Event):
    """Message class 'ComputeCoordinate_Event'."""

    __slots__ = [
        '_info',
        '_request',
        '_response',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'info': 'service_msgs/ServiceEventInfo',
        'request': 'sequence<reseq_interfaces/ComputeCoordinate_Request, 1>',
        'response': 'sequence<reseq_interfaces/ComputeCoordinate_Response, 1>',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['service_msgs', 'msg'], 'ServiceEventInfo'),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.NamespacedType(['reseq_interfaces', 'srv'], 'ComputeCoordinate_Request'), 1),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.NamespacedType(['reseq_interfaces', 'srv'], 'ComputeCoordinate_Response'), 1),  # noqa: E501
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
            from reseq_interfaces.srv import ComputeCoordinate_Request
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
                 all(isinstance(v, ComputeCoordinate_Request) for v in value) and
                 True), \
                "The 'request' field must be a set or sequence with length <= 1 and each value of type 'ComputeCoordinate_Request'"
        self._request = value

    @builtins.property
    def response(self):
        """Message field 'response'."""
        return self._response

    @response.setter
    def response(self, value):
        if self._check_fields:
            from reseq_interfaces.srv import ComputeCoordinate_Response
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
                 all(isinstance(v, ComputeCoordinate_Response) for v in value) and
                 True), \
                "The 'response' field must be a set or sequence with length <= 1 and each value of type 'ComputeCoordinate_Response'"
        self._response = value


class Metaclass_ComputeCoordinate(type):
    """Metaclass of service 'ComputeCoordinate'."""

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
                'reseq_interfaces.srv.ComputeCoordinate')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__compute_coordinate

            from reseq_interfaces.srv import _compute_coordinate
            if _compute_coordinate.Metaclass_ComputeCoordinate_Request._TYPE_SUPPORT is None:
                _compute_coordinate.Metaclass_ComputeCoordinate_Request.__import_type_support__()
            if _compute_coordinate.Metaclass_ComputeCoordinate_Response._TYPE_SUPPORT is None:
                _compute_coordinate.Metaclass_ComputeCoordinate_Response.__import_type_support__()
            if _compute_coordinate.Metaclass_ComputeCoordinate_Event._TYPE_SUPPORT is None:
                _compute_coordinate.Metaclass_ComputeCoordinate_Event.__import_type_support__()


class ComputeCoordinate(metaclass=Metaclass_ComputeCoordinate):
    from reseq_interfaces.srv._compute_coordinate import ComputeCoordinate_Request as Request
    from reseq_interfaces.srv._compute_coordinate import ComputeCoordinate_Response as Response
    from reseq_interfaces.srv._compute_coordinate import ComputeCoordinate_Event as Event

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
