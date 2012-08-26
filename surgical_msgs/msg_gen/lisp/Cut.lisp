; Auto-generated. Do not edit!


(cl:in-package surgical_msgs-msg)


;//! \htmlinclude Cut.msg.html

(cl:defclass <Cut> (roslisp-msg-protocol:ros-message)
  ((nodes
    :reader nodes
    :initarg :nodes
    :type (cl:vector surgical_msgs-msg:Hole)
   :initform (cl:make-array 0 :element-type 'surgical_msgs-msg:Hole :initial-element (cl:make-instance 'surgical_msgs-msg:Hole))))
)

(cl:defclass Cut (<Cut>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Cut>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Cut)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name surgical_msgs-msg:<Cut> is deprecated: use surgical_msgs-msg:Cut instead.")))

(cl:ensure-generic-function 'nodes-val :lambda-list '(m))
(cl:defmethod nodes-val ((m <Cut>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader surgical_msgs-msg:nodes-val is deprecated.  Use surgical_msgs-msg:nodes instead.")
  (nodes m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Cut>) ostream)
  "Serializes a message object of type '<Cut>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'nodes))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'nodes))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Cut>) istream)
  "Deserializes a message object of type '<Cut>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'nodes) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'nodes)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'surgical_msgs-msg:Hole))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Cut>)))
  "Returns string type for a message object of type '<Cut>"
  "surgical_msgs/Cut")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Cut)))
  "Returns string type for a message object of type 'Cut"
  "surgical_msgs/Cut")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Cut>)))
  "Returns md5sum for a message object of type '<Cut>"
  "9df5bac2e65b67663e436bfe4fb0e3e4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Cut)))
  "Returns md5sum for a message object of type 'Cut"
  "9df5bac2e65b67663e436bfe4fb0e3e4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Cut>)))
  "Returns full string definition for message of type '<Cut>"
  (cl:format cl:nil "# defines an array for 3D points.~%Hole[] nodes~%~%================================================================================~%MSG: surgical_msgs/Hole~%# Defines the various attributes of a HOLE~%# ----------------------------------------~%#~%# 1. PT : the 3D location of the HOLE~%geometry_msgs/Point pt~%~%# 2. X_IDX : the x-index of the location of the point in the point-cloud~%int32 x_idx~%~%# 3. Y_IDX : the y-index of the location of the point in the point-cloud~%int32 y_idx~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Cut)))
  "Returns full string definition for message of type 'Cut"
  (cl:format cl:nil "# defines an array for 3D points.~%Hole[] nodes~%~%================================================================================~%MSG: surgical_msgs/Hole~%# Defines the various attributes of a HOLE~%# ----------------------------------------~%#~%# 1. PT : the 3D location of the HOLE~%geometry_msgs/Point pt~%~%# 2. X_IDX : the x-index of the location of the point in the point-cloud~%int32 x_idx~%~%# 3. Y_IDX : the y-index of the location of the point in the point-cloud~%int32 y_idx~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Cut>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'nodes) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Cut>))
  "Converts a ROS message object to a list"
  (cl:list 'Cut
    (cl:cons ':nodes (nodes msg))
))
