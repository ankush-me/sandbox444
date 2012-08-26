; Auto-generated. Do not edit!


(cl:in-package surgical_msgs-msg)


;//! \htmlinclude InitInfo.msg.html

(cl:defclass <InitInfo> (roslisp-msg-protocol:ros-message)
  ((holes
    :reader holes
    :initarg :holes
    :type (cl:vector surgical_msgs-msg:Hole)
   :initform (cl:make-array 0 :element-type 'surgical_msgs-msg:Hole :initial-element (cl:make-instance 'surgical_msgs-msg:Hole)))
   (cuts
    :reader cuts
    :initarg :cuts
    :type (cl:vector surgical_msgs-msg:Cut)
   :initform (cl:make-array 0 :element-type 'surgical_msgs-msg:Cut :initial-element (cl:make-instance 'surgical_msgs-msg:Cut)))
   (cloud
    :reader cloud
    :initarg :cloud
    :type sensor_msgs-msg:PointCloud2
    :initform (cl:make-instance 'sensor_msgs-msg:PointCloud2)))
)

(cl:defclass InitInfo (<InitInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <InitInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'InitInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name surgical_msgs-msg:<InitInfo> is deprecated: use surgical_msgs-msg:InitInfo instead.")))

(cl:ensure-generic-function 'holes-val :lambda-list '(m))
(cl:defmethod holes-val ((m <InitInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader surgical_msgs-msg:holes-val is deprecated.  Use surgical_msgs-msg:holes instead.")
  (holes m))

(cl:ensure-generic-function 'cuts-val :lambda-list '(m))
(cl:defmethod cuts-val ((m <InitInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader surgical_msgs-msg:cuts-val is deprecated.  Use surgical_msgs-msg:cuts instead.")
  (cuts m))

(cl:ensure-generic-function 'cloud-val :lambda-list '(m))
(cl:defmethod cloud-val ((m <InitInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader surgical_msgs-msg:cloud-val is deprecated.  Use surgical_msgs-msg:cloud instead.")
  (cloud m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <InitInfo>) ostream)
  "Serializes a message object of type '<InitInfo>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'holes))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'holes))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'cuts))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'cuts))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'cloud) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <InitInfo>) istream)
  "Deserializes a message object of type '<InitInfo>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'holes) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'holes)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'surgical_msgs-msg:Hole))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'cuts) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'cuts)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'surgical_msgs-msg:Cut))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'cloud) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<InitInfo>)))
  "Returns string type for a message object of type '<InitInfo>"
  "surgical_msgs/InitInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'InitInfo)))
  "Returns string type for a message object of type 'InitInfo"
  "surgical_msgs/InitInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<InitInfo>)))
  "Returns md5sum for a message object of type '<InitInfo>"
  "47565f315bb0e1a0dffdb25437d7b98a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'InitInfo)))
  "Returns md5sum for a message object of type 'InitInfo"
  "47565f315bb0e1a0dffdb25437d7b98a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<InitInfo>)))
  "Returns full string definition for message of type '<InitInfo>"
  (cl:format cl:nil "# Defines CUTS and HOLES and the reference point-cloud~%~%# Holes are a bunch of 3D points~%Hole[] holes~%~%# An array of cut is cuts. A cut is an array of 3D points.~%Cut[] cuts~%~%# The cloud on which the holes and cuts are defined~%sensor_msgs/PointCloud2 cloud~%================================================================================~%MSG: surgical_msgs/Hole~%# Defines the various attributes of a HOLE~%# ----------------------------------------~%#~%# 1. PT : the 3D location of the HOLE~%geometry_msgs/Point pt~%~%# 2. X_IDX : the x-index of the location of the point in the point-cloud~%int32 x_idx~%~%# 3. Y_IDX : the y-index of the location of the point in the point-cloud~%int32 y_idx~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: surgical_msgs/Cut~%# defines an array for 3D points.~%Hole[] nodes~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'InitInfo)))
  "Returns full string definition for message of type 'InitInfo"
  (cl:format cl:nil "# Defines CUTS and HOLES and the reference point-cloud~%~%# Holes are a bunch of 3D points~%Hole[] holes~%~%# An array of cut is cuts. A cut is an array of 3D points.~%Cut[] cuts~%~%# The cloud on which the holes and cuts are defined~%sensor_msgs/PointCloud2 cloud~%================================================================================~%MSG: surgical_msgs/Hole~%# Defines the various attributes of a HOLE~%# ----------------------------------------~%#~%# 1. PT : the 3D location of the HOLE~%geometry_msgs/Point pt~%~%# 2. X_IDX : the x-index of the location of the point in the point-cloud~%int32 x_idx~%~%# 3. Y_IDX : the y-index of the location of the point in the point-cloud~%int32 y_idx~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: surgical_msgs/Cut~%# defines an array for 3D points.~%Hole[] nodes~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <InitInfo>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'holes) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'cuts) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'cloud))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <InitInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'InitInfo
    (cl:cons ':holes (holes msg))
    (cl:cons ':cuts (cuts msg))
    (cl:cons ':cloud (cloud msg))
))
