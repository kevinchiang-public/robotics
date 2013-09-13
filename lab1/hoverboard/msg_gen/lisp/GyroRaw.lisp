; Auto-generated. Do not edit!


(cl:in-package hoverboard-msg)


;//! \htmlinclude GyroRaw.msg.html

(cl:defclass <GyroRaw> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (angle
    :reader angle
    :initarg :angle
    :type cl:fixnum
    :initform 0)
   (rate
    :reader rate
    :initarg :rate
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GyroRaw (<GyroRaw>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GyroRaw>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GyroRaw)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hoverboard-msg:<GyroRaw> is deprecated: use hoverboard-msg:GyroRaw instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <GyroRaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hoverboard-msg:header-val is deprecated.  Use hoverboard-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <GyroRaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hoverboard-msg:angle-val is deprecated.  Use hoverboard-msg:angle instead.")
  (angle m))

(cl:ensure-generic-function 'rate-val :lambda-list '(m))
(cl:defmethod rate-val ((m <GyroRaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hoverboard-msg:rate-val is deprecated.  Use hoverboard-msg:rate instead.")
  (rate m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GyroRaw>) ostream)
  "Serializes a message object of type '<GyroRaw>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'angle)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'rate)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GyroRaw>) istream)
  "Deserializes a message object of type '<GyroRaw>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'angle) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'rate) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GyroRaw>)))
  "Returns string type for a message object of type '<GyroRaw>"
  "hoverboard/GyroRaw")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GyroRaw)))
  "Returns string type for a message object of type 'GyroRaw"
  "hoverboard/GyroRaw")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GyroRaw>)))
  "Returns md5sum for a message object of type '<GyroRaw>"
  "cf76f9b1b2b3e94db06d36db0a223cd3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GyroRaw)))
  "Returns md5sum for a message object of type 'GyroRaw"
  "cf76f9b1b2b3e94db06d36db0a223cd3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GyroRaw>)))
  "Returns full string definition for message of type '<GyroRaw>"
  (cl:format cl:nil "Header header~%~%# Gyro reported angle in degrees 0 to 360 degrees~%int16 angle~%# Gyro reported rate in degrees per second~%int16 rate~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GyroRaw)))
  "Returns full string definition for message of type 'GyroRaw"
  (cl:format cl:nil "Header header~%~%# Gyro reported angle in degrees 0 to 360 degrees~%int16 angle~%# Gyro reported rate in degrees per second~%int16 rate~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GyroRaw>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GyroRaw>))
  "Converts a ROS message object to a list"
  (cl:list 'GyroRaw
    (cl:cons ':header (header msg))
    (cl:cons ':angle (angle msg))
    (cl:cons ':rate (rate msg))
))
