; Auto-generated. Do not edit!


(cl:in-package hovercraft-msg)


;//! \htmlinclude BatteryVoltage.msg.html

(cl:defclass <BatteryVoltage> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (voltage
    :reader voltage
    :initarg :voltage
    :type cl:float
    :initform 0.0))
)

(cl:defclass BatteryVoltage (<BatteryVoltage>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BatteryVoltage>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BatteryVoltage)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hovercraft-msg:<BatteryVoltage> is deprecated: use hovercraft-msg:BatteryVoltage instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <BatteryVoltage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hovercraft-msg:header-val is deprecated.  Use hovercraft-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'voltage-val :lambda-list '(m))
(cl:defmethod voltage-val ((m <BatteryVoltage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hovercraft-msg:voltage-val is deprecated.  Use hovercraft-msg:voltage instead.")
  (voltage m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BatteryVoltage>) ostream)
  "Serializes a message object of type '<BatteryVoltage>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'voltage))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BatteryVoltage>) istream)
  "Deserializes a message object of type '<BatteryVoltage>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'voltage) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BatteryVoltage>)))
  "Returns string type for a message object of type '<BatteryVoltage>"
  "hovercraft/BatteryVoltage")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BatteryVoltage)))
  "Returns string type for a message object of type 'BatteryVoltage"
  "hovercraft/BatteryVoltage")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BatteryVoltage>)))
  "Returns md5sum for a message object of type '<BatteryVoltage>"
  "eaf4d0e55ee7ea889fa473746e54d896")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BatteryVoltage)))
  "Returns md5sum for a message object of type 'BatteryVoltage"
  "eaf4d0e55ee7ea889fa473746e54d896")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BatteryVoltage>)))
  "Returns full string definition for message of type '<BatteryVoltage>"
  (cl:format cl:nil "Header header~%~%# In volts~%float64 voltage~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BatteryVoltage)))
  "Returns full string definition for message of type 'BatteryVoltage"
  (cl:format cl:nil "Header header~%~%# In volts~%float64 voltage~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BatteryVoltage>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BatteryVoltage>))
  "Converts a ROS message object to a list"
  (cl:list 'BatteryVoltage
    (cl:cons ':header (header msg))
    (cl:cons ':voltage (voltage msg))
))
