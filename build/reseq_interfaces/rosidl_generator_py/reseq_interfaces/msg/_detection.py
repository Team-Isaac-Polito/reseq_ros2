# generated from rosidl_generator_py/resource/_idl.py.em
# with input from reseq_interfaces:msg/Detection.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Detection(type):
    """Metaclass of message 'Detection'."""

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
                'reseq_interfaces.msg.Detection')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__detection
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__detection
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__detection
            cls._TYPE_SUPPORT = module.type_support_msg__msg__detection
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__detection

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Detection(metaclass=Metaclass_Detection):
    """Message class 'Detection'."""

    __slots__ = [
        '_header',
        '_detection',
        '_type',
        '_name',
        '_robot',
        '_mode',
        '_confidence',
        '_xmin',
        '_ymin',
        '_width',
        '_height',
        '_depth_center',
        '_camera_frame',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'detection': 'int32',
        'type': 'string',
        'name': 'string',
        'robot': 'string',
        'mode': 'string',
        'confidence': 'float',
        'xmin': 'int32',
        'ymin': 'int32',
        'width': 'uint32',
        'height': 'uint32',
        'depth_center': 'float',
        'camera_frame': 'string',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
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
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.detection = kwargs.get('detection', int())
        self.type = kwargs.get('type', str())
        self.name = kwargs.get('name', str())
        self.robot = kwargs.get('robot', str())
        self.mode = kwargs.get('mode', str())
        self.confidence = kwargs.get('confidence', float())
        self.xmin = kwargs.get('xmin', int())
        self.ymin = kwargs.get('ymin', int())
        self.width = kwargs.get('width', int())
        self.height = kwargs.get('height', int())
        self.depth_center = kwargs.get('depth_center', float())
        self.camera_frame = kwargs.get('camera_frame', str())

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
        if self.header != other.header:
            return False
        if self.detection != other.detection:
            return False
        if self.type != other.type:
            return False
        if self.name != other.name:
            return False
        if self.robot != other.robot:
            return False
        if self.mode != other.mode:
            return False
        if self.confidence != other.confidence:
            return False
        if self.xmin != other.xmin:
            return False
        if self.ymin != other.ymin:
            return False
        if self.width != other.width:
            return False
        if self.height != other.height:
            return False
        if self.depth_center != other.depth_center:
            return False
        if self.camera_frame != other.camera_frame:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if self._check_fields:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def detection(self):
        """Message field 'detection'."""
        return self._detection

    @detection.setter
    def detection(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'detection' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'detection' field must be an integer in [-2147483648, 2147483647]"
        self._detection = value

    @builtins.property  # noqa: A003
    def type(self):  # noqa: A003
        """Message field 'type'."""
        return self._type

    @type.setter  # noqa: A003
    def type(self, value):  # noqa: A003
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'type' field must be of type 'str'"
        self._type = value

    @builtins.property
    def name(self):
        """Message field 'name'."""
        return self._name

    @name.setter
    def name(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'name' field must be of type 'str'"
        self._name = value

    @builtins.property
    def robot(self):
        """Message field 'robot'."""
        return self._robot

    @robot.setter
    def robot(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'robot' field must be of type 'str'"
        self._robot = value

    @builtins.property
    def mode(self):
        """Message field 'mode'."""
        return self._mode

    @mode.setter
    def mode(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'mode' field must be of type 'str'"
        self._mode = value

    @builtins.property
    def confidence(self):
        """Message field 'confidence'."""
        return self._confidence

    @confidence.setter
    def confidence(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'confidence' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'confidence' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._confidence = value

    @builtins.property
    def xmin(self):
        """Message field 'xmin'."""
        return self._xmin

    @xmin.setter
    def xmin(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'xmin' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'xmin' field must be an integer in [-2147483648, 2147483647]"
        self._xmin = value

    @builtins.property
    def ymin(self):
        """Message field 'ymin'."""
        return self._ymin

    @ymin.setter
    def ymin(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'ymin' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'ymin' field must be an integer in [-2147483648, 2147483647]"
        self._ymin = value

    @builtins.property
    def width(self):
        """Message field 'width'."""
        return self._width

    @width.setter
    def width(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'width' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'width' field must be an unsigned integer in [0, 4294967295]"
        self._width = value

    @builtins.property
    def height(self):
        """Message field 'height'."""
        return self._height

    @height.setter
    def height(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'height' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'height' field must be an unsigned integer in [0, 4294967295]"
        self._height = value

    @builtins.property
    def depth_center(self):
        """Message field 'depth_center'."""
        return self._depth_center

    @depth_center.setter
    def depth_center(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'depth_center' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'depth_center' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._depth_center = value

    @builtins.property
    def camera_frame(self):
        """Message field 'camera_frame'."""
        return self._camera_frame

    @camera_frame.setter
    def camera_frame(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'camera_frame' field must be of type 'str'"
        self._camera_frame = value
