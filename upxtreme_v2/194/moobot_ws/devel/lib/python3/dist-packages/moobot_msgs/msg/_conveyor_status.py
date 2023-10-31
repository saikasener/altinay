# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from moobot_msgs/conveyor_status.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class conveyor_status(genpy.Message):
  _md5sum = "a8462790025e5d19ec5df8dbb71cb071"
  _type = "moobot_msgs/conveyor_status"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """bool load_conveyor
bool unload_conveyor"""
  __slots__ = ['load_conveyor','unload_conveyor']
  _slot_types = ['bool','bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       load_conveyor,unload_conveyor

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(conveyor_status, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.load_conveyor is None:
        self.load_conveyor = False
      if self.unload_conveyor is None:
        self.unload_conveyor = False
    else:
      self.load_conveyor = False
      self.unload_conveyor = False

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
      _x = self
      buff.write(_get_struct_2B().pack(_x.load_conveyor, _x.unload_conveyor))
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
      _x = self
      start = end
      end += 2
      (_x.load_conveyor, _x.unload_conveyor,) = _get_struct_2B().unpack(str[start:end])
      self.load_conveyor = bool(self.load_conveyor)
      self.unload_conveyor = bool(self.unload_conveyor)
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
      _x = self
      buff.write(_get_struct_2B().pack(_x.load_conveyor, _x.unload_conveyor))
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
      _x = self
      start = end
      end += 2
      (_x.load_conveyor, _x.unload_conveyor,) = _get_struct_2B().unpack(str[start:end])
      self.load_conveyor = bool(self.load_conveyor)
      self.unload_conveyor = bool(self.unload_conveyor)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2B = None
def _get_struct_2B():
    global _struct_2B
    if _struct_2B is None:
        _struct_2B = struct.Struct("<2B")
    return _struct_2B
