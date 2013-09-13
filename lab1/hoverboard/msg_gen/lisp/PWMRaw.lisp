; Auto-generated. Do not edit!


(cl:in-package hoverboard-msg)


;//! \htmlinclude PWMRaw.msg.html

(cl:defclass <PWMRaw> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (channel
    :reader channel
    :initarg :channel
    :type cl:fixnum
    :initform 0)
   (pwm
    :reader pwm
    :initarg :pwm
    :type cl:integer
    :initform 0))
)

(cl:defclass PWMRaw (<PWMRaw>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PWMRaw>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PWMRaw)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hoverboard-msg:<PWMRaw> is deprecated: use hoverboard-msg:PWMRaw instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <PWMRaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hoverboard-msg:header-val is deprecated.  Use hoverboard-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'channel-val :lambda-list '(m))
(cl:defmethod channel-val ((m <PWMRaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hoverboard-msg:channel-val is deprecated.  Use hoverboard-msg:channel instead.")
  (channel m))

(cl:ensure-generic-function 'pwm-val :lambda-list '(m))
(cl:defmethod pwm-val ((m <PWMRaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hoverboard-msg:pwm-val is deprecated.  Use hoverboard-msg:pwm instead.")
  (pwm m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PWMRaw>) ostream)
  "Serializes a message object of type '<PWMRaw>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'channel)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'pwm)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PWMRaw>) istream)
  "Deserializes a message object of type '<PWMRaw>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'channel) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'pwm) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PWMRaw>)))
  "Returns string type for a message object of type '<PWMRaw>"
  "hoverboard/PWMRaw")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PWMRaw)))
  "Returns string type for a message object of type 'PWMRaw"
  "hoverboard/PWMRaw")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PWMRaw>)))
  "Returns md5sum for a message object of type '<PWMRaw>"
  "1e11724542b6627b3863c96bbe05f93e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PWMRaw)))
  "Returns md5sum for a message object of type 'PWMRaw"
  "1e11724542b6627b3863c96bbe05f93e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PWMRaw>)))
  "Returns full string definition for message of type '<PWMRaw>"
  (cl:format cl:nil "Header header~%~%# The pwm channel to set (0-5)~%int8 channel~%~%# PWM value setting from 0 to 100 (inclusive), negative value sets~%# state to high-z (no output).~%int32 pwm~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PWMRaw)))
  "Returns full string definition for message of type 'PWMRaw"
  (cl:format cl:nil "Header header~%~%# The pwm channel to set (0-5)~%int8 channel~%~%# PWM value setting from 0 to 100 (inclusive), negative value sets~%# state to high-z (no output).~%int32 pwm~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PWMRaw>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PWMRaw>))
  "Converts a ROS message object to a list"
  (cl:list 'PWMRaw
    (cl:cons ':header (header msg))
    (cl:cons ':channel (channel msg))
    (cl:cons ':pwm (pwm msg))
))
