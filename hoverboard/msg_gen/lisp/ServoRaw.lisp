; Auto-generated. Do not edit!


(cl:in-package hoverboard-msg)


;//! \htmlinclude ServoRaw.msg.html

(cl:defclass <ServoRaw> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (port
    :reader port
    :initarg :port
    :type cl:fixnum
    :initform 0)
   (value
    :reader value
    :initarg :value
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ServoRaw (<ServoRaw>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ServoRaw>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ServoRaw)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hoverboard-msg:<ServoRaw> is deprecated: use hoverboard-msg:ServoRaw instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ServoRaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hoverboard-msg:header-val is deprecated.  Use hoverboard-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'port-val :lambda-list '(m))
(cl:defmethod port-val ((m <ServoRaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hoverboard-msg:port-val is deprecated.  Use hoverboard-msg:port instead.")
  (port m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <ServoRaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hoverboard-msg:value-val is deprecated.  Use hoverboard-msg:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ServoRaw>) ostream)
  "Serializes a message object of type '<ServoRaw>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'port)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'value)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ServoRaw>) istream)
  "Deserializes a message object of type '<ServoRaw>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'port) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'value) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ServoRaw>)))
  "Returns string type for a message object of type '<ServoRaw>"
  "hoverboard/ServoRaw")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ServoRaw)))
  "Returns string type for a message object of type 'ServoRaw"
  "hoverboard/ServoRaw")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ServoRaw>)))
  "Returns md5sum for a message object of type '<ServoRaw>"
  "cf1c9d17f7bbedbe8dd2c29cdb7700f8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ServoRaw)))
  "Returns md5sum for a message object of type 'ServoRaw"
  "cf1c9d17f7bbedbe8dd2c29cdb7700f8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ServoRaw>)))
  "Returns full string definition for message of type '<ServoRaw>"
  (cl:format cl:nil "Header header~%~%# The servo port 0 through 5 inclusive~%int8 port~%# The value to send 0 to 100 inclusive~%int8 value~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ServoRaw)))
  "Returns full string definition for message of type 'ServoRaw"
  (cl:format cl:nil "Header header~%~%# The servo port 0 through 5 inclusive~%int8 port~%# The value to send 0 to 100 inclusive~%int8 value~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ServoRaw>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ServoRaw>))
  "Converts a ROS message object to a list"
  (cl:list 'ServoRaw
    (cl:cons ':header (header msg))
    (cl:cons ':port (port msg))
    (cl:cons ':value (value msg))
))
