; Auto-generated. Do not edit!


(cl:in-package surgical_msgs-msg)


;//! \htmlinclude Hole.msg.html

(cl:defclass <Hole> (roslisp-msg-protocol:ros-message)
  ((pt
    :reader pt
    :initarg :pt
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (x_idx
    :reader x_idx
    :initarg :x_idx
    :type cl:integer
    :initform 0)
   (y_idx
    :reader y_idx
    :initarg :y_idx
    :type cl:integer
    :initform 0))
)

(cl:defclass Hole (<Hole>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Hole>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Hole)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name surgical_msgs-msg:<Hole> is deprecated: use surgical_msgs-msg:Hole instead.")))

(cl:ensure-generic-function 'pt-val :lambda-list '(m))
(cl:defmethod pt-val ((m <Hole>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader surgical_msgs-msg:pt-val is deprecated.  Use surgical_msgs-msg:pt instead.")
  (pt m))

(cl:ensure-generic-function 'x_idx-val :lambda-list '(m))
(cl:defmethod x_idx-val ((m <Hole>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader surgical_msgs-msg:x_idx-val is deprecated.  Use surgical_msgs-msg:x_idx instead.")
  (x_idx m))

(cl:ensure-generic-function 'y_idx-val :lambda-list '(m))
(cl:defmethod y_idx-val ((m <Hole>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader surgical_msgs-msg:y_idx-val is deprecated.  Use surgical_msgs-msg:y_idx instead.")
  (y_idx m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Hole>) ostream)
  "Serializes a message object of type '<Hole>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pt) ostream)
  (cl:let* ((signed (cl:slot-value msg 'x_idx)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'y_idx)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Hole>) istream)
  "Deserializes a message object of type '<Hole>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pt) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'x_idx) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'y_idx) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Hole>)))
  "Returns string type for a message object of type '<Hole>"
  "surgical_msgs/Hole")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Hole)))
  "Returns string type for a message object of type 'Hole"
  "surgical_msgs/Hole")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Hole>)))
  "Returns md5sum for a message object of type '<Hole>"
  "64cb44aecb568b09a90194264446d6fe")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Hole)))
  "Returns md5sum for a message object of type 'Hole"
  "64cb44aecb568b09a90194264446d6fe")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Hole>)))
  "Returns full string definition for message of type '<Hole>"
  (cl:format cl:nil "# Defines the various attributes of a HOLE~%# ----------------------------------------~%#~%# 1. PT : the 3D location of the HOLE~%geometry_msgs/Point pt~%~%# 2. X_IDX : the x-index of the location of the point in the point-cloud~%int32 x_idx~%~%# 3. Y_IDX : the y-index of the location of the point in the point-cloud~%int32 y_idx~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Hole)))
  "Returns full string definition for message of type 'Hole"
  (cl:format cl:nil "# Defines the various attributes of a HOLE~%# ----------------------------------------~%#~%# 1. PT : the 3D location of the HOLE~%geometry_msgs/Point pt~%~%# 2. X_IDX : the x-index of the location of the point in the point-cloud~%int32 x_idx~%~%# 3. Y_IDX : the y-index of the location of the point in the point-cloud~%int32 y_idx~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Hole>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pt))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Hole>))
  "Converts a ROS message object to a list"
  (cl:list 'Hole
    (cl:cons ':pt (pt msg))
    (cl:cons ':x_idx (x_idx msg))
    (cl:cons ':y_idx (y_idx msg))
))
