"""autogenerated by genpy from surgical_msgs/Cut.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import surgical_msgs.msg
import geometry_msgs.msg

class Cut(genpy.Message):
  _md5sum = "9df5bac2e65b67663e436bfe4fb0e3e4"
  _type = "surgical_msgs/Cut"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """# defines an array for 3D points.
Hole[] nodes

================================================================================
MSG: surgical_msgs/Hole
# Defines the various attributes of a HOLE
# ----------------------------------------
#
# 1. PT : the 3D location of the HOLE
geometry_msgs/Point pt

# 2. X_IDX : the x-index of the location of the point in the point-cloud
int32 x_idx

# 3. Y_IDX : the y-index of the location of the point in the point-cloud
int32 y_idx


================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

"""
  __slots__ = ['nodes']
  _slot_types = ['surgical_msgs/Hole[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       nodes

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Cut, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.nodes is None:
        self.nodes = []
    else:
      self.nodes = []

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
      length = len(self.nodes)
      buff.write(_struct_I.pack(length))
      for val1 in self.nodes:
        _v1 = val1.pt
        _x = _v1
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _x = val1
        buff.write(_struct_2i.pack(_x.x_idx, _x.y_idx))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.nodes is None:
        self.nodes = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.nodes = []
      for i in range(0, length):
        val1 = surgical_msgs.msg.Hole()
        _v2 = val1.pt
        _x = _v2
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _x = val1
        start = end
        end += 8
        (_x.x_idx, _x.y_idx,) = _struct_2i.unpack(str[start:end])
        self.nodes.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      length = len(self.nodes)
      buff.write(_struct_I.pack(length))
      for val1 in self.nodes:
        _v3 = val1.pt
        _x = _v3
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _x = val1
        buff.write(_struct_2i.pack(_x.x_idx, _x.y_idx))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.nodes is None:
        self.nodes = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.nodes = []
      for i in range(0, length):
        val1 = surgical_msgs.msg.Hole()
        _v4 = val1.pt
        _x = _v4
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _x = val1
        start = end
        end += 8
        (_x.x_idx, _x.y_idx,) = _struct_2i.unpack(str[start:end])
        self.nodes.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_2i = struct.Struct("<2i")
_struct_3d = struct.Struct("<3d")
