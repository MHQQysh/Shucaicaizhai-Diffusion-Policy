# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from bmirobot_msg/Robot_distance.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class Robot_distance(genpy.Message):
  _md5sum = "68c6629711bc8cca4c5688c4b92123ab"
  _type = "bmirobot_msg/Robot_distance"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """uint16[25] proximity
uint16[25] realdistance
"""
  __slots__ = ['proximity','realdistance']
  _slot_types = ['uint16[25]','uint16[25]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       proximity,realdistance

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Robot_distance, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.proximity is None:
        self.proximity = [0] * 25
      if self.realdistance is None:
        self.realdistance = [0] * 25
    else:
      self.proximity = [0] * 25
      self.realdistance = [0] * 25

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
      buff.write(_get_struct_25H().pack(*self.proximity))
      buff.write(_get_struct_25H().pack(*self.realdistance))
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
      end += 50
      self.proximity = _get_struct_25H().unpack(str[start:end])
      start = end
      end += 50
      self.realdistance = _get_struct_25H().unpack(str[start:end])
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
      buff.write(self.proximity.tostring())
      buff.write(self.realdistance.tostring())
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
      end += 50
      self.proximity = numpy.frombuffer(str[start:end], dtype=numpy.uint16, count=25)
      start = end
      end += 50
      self.realdistance = numpy.frombuffer(str[start:end], dtype=numpy.uint16, count=25)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_25H = None
def _get_struct_25H():
    global _struct_25H
    if _struct_25H is None:
        _struct_25H = struct.Struct("<25H")
    return _struct_25H
