# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from bmirobot_msg/motorv2_ctr.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class motorv2_ctr(genpy.Message):
  _md5sum = "28290c9afcbb7e417ea5e1d8b11ed1b4"
  _type = "bmirobot_msg/motorv2_ctr"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """int8[8] mt_operation_mode
int32[8] mt_position_goal
int32[8] mt_speed_goal
int32[8] mt_current_goal
int32[8] mt_pwm_goal
"""
  __slots__ = ['mt_operation_mode','mt_position_goal','mt_speed_goal','mt_current_goal','mt_pwm_goal']
  _slot_types = ['int8[8]','int32[8]','int32[8]','int32[8]','int32[8]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       mt_operation_mode,mt_position_goal,mt_speed_goal,mt_current_goal,mt_pwm_goal

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(motorv2_ctr, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.mt_operation_mode is None:
        self.mt_operation_mode = [0] * 8
      if self.mt_position_goal is None:
        self.mt_position_goal = [0] * 8
      if self.mt_speed_goal is None:
        self.mt_speed_goal = [0] * 8
      if self.mt_current_goal is None:
        self.mt_current_goal = [0] * 8
      if self.mt_pwm_goal is None:
        self.mt_pwm_goal = [0] * 8
    else:
      self.mt_operation_mode = [0] * 8
      self.mt_position_goal = [0] * 8
      self.mt_speed_goal = [0] * 8
      self.mt_current_goal = [0] * 8
      self.mt_pwm_goal = [0] * 8

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      buff.write(_get_struct_8b().pack(*self.mt_operation_mode))
      buff.write(_get_struct_8i().pack(*self.mt_position_goal))
      buff.write(_get_struct_8i().pack(*self.mt_speed_goal))
      buff.write(_get_struct_8i().pack(*self.mt_current_goal))
      buff.write(_get_struct_8i().pack(*self.mt_pwm_goal))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      start = end
      end += 8
      self.mt_operation_mode = _get_struct_8b().unpack(str[start:end])
      start = end
      end += 32
      self.mt_position_goal = _get_struct_8i().unpack(str[start:end])
      start = end
      end += 32
      self.mt_speed_goal = _get_struct_8i().unpack(str[start:end])
      start = end
      end += 32
      self.mt_current_goal = _get_struct_8i().unpack(str[start:end])
      start = end
      end += 32
      self.mt_pwm_goal = _get_struct_8i().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      buff.write(self.mt_operation_mode.tostring())
      buff.write(self.mt_position_goal.tostring())
      buff.write(self.mt_speed_goal.tostring())
      buff.write(self.mt_current_goal.tostring())
      buff.write(self.mt_pwm_goal.tostring())
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      start = end
      end += 8
      self.mt_operation_mode = numpy.frombuffer(str[start:end], dtype=numpy.int8, count=8)
      start = end
      end += 32
      self.mt_position_goal = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=8)
      start = end
      end += 32
      self.mt_speed_goal = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=8)
      start = end
      end += 32
      self.mt_current_goal = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=8)
      start = end
      end += 32
      self.mt_pwm_goal = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=8)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_8b = None
def _get_struct_8b():
    global _struct_8b
    if _struct_8b is None:
        _struct_8b = struct.Struct("<8b")
    return _struct_8b
_struct_8i = None
def _get_struct_8i():
    global _struct_8i
    if _struct_8i is None:
        _struct_8i = struct.Struct("<8i")
    return _struct_8i
