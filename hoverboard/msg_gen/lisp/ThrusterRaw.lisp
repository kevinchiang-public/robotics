; Auto-generated. Do not edit!


(cl:in-package hoverboard-msg)


;//! \htmlinclude ThrusterRaw.msg.html

(cl:defclass <ThrusterRaw> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (thruster1
    :reader thruster1
    :initarg :thruster1
    :type cl:fixnum
    :initform 0)
   (thruster2
    :reader thruster2
    :initarg :thruster2
    :type cl:fixnum
    :initform 0)
   (thruster3
    :reader thruster3
    :initarg :thruster3
    :type cl:fixnum
    :initform 0)
   (thruster4
    :reader thruster4
    :initarg :thruster4
    :type cl:fixnum
    :initform 0)
   (thruster5
    :reader thruster5
    :initarg :thruster5
    :type cl:fixnum
    :initform 0)
   (thruster6
    :reader thruster6
    :initarg :thruster6
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ThrusterRaw (<ThrusterRaw>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ThrusterRaw>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ThrusterRaw)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hoverboard-msg:<ThrusterRaw> is deprecated: use hoverboard-msg:ThrusterRaw instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ThrusterRaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hoverboard-msg:header-val is deprecated.  Use hoverboard-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'thruster1-val :lambda-list '(m))
(cl:defmethod thruster1-val ((m <ThrusterRaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hoverboard-msg:thruster1-val is deprecated.  Use hoverboard-msg:thruster1 instead.")
  (thruster1 m))

(cl:ensure-generic-function 'thruster2-val :lambda-list '(m))
(cl:defmethod thruster2-val ((m <ThrusterRaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hoverboard-msg:thruster2-val is deprecated.  Use hoverboard-msg:thruster2 instead.")
  (thruster2 m))

(cl:ensure-generic-function 'thruster3-val :lambda-list '(m))
(cl:defmethod thruster3-val ((m <ThrusterRaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hoverboard-msg:thruster3-val is deprecated.  Use hoverboard-msg:thruster3 instead.")
  (thruster3 m))

(cl:ensure-generic-function 'thruster4-val :lambda-list '(m))
(cl:defmethod thruster4-val ((m <ThrusterRaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hoverboard-msg:thruster4-val is deprecated.  Use hoverboard-msg:thruster4 instead.")
  (thruster4 m))

(cl:ensure-generic-function 'thruster5-val :lambda-list '(m))
(cl:defmethod thruster5-val ((m <ThrusterRaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hoverboard-msg:thruster5-val is deprecated.  Use hoverboard-msg:thruster5 instead.")
  (thruster5 m))

(cl:ensure-generic-function 'thruster6-val :lambda-list '(m))
(cl:defmethod thruster6-val ((m <ThrusterRaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hoverboard-msg:thruster6-val is deprecated.  Use hoverboard-msg:thruster6 instead.")
  (thruster6 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ThrusterRaw>) ostream)
  "Serializes a message object of type '<ThrusterRaw>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'thruster1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'thruster2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'thruster3)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'thruster4)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'thruster5)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'thruster6)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ThrusterRaw>) istream)
  "Deserializes a message object of type '<ThrusterRaw>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'thruster1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'thruster2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'thruster3)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'thruster4)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'thruster5)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'thruster6)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ThrusterRaw>)))
  "Returns string type for a message object of type '<ThrusterRaw>"
  "hoverboard/ThrusterRaw")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ThrusterRaw)))
  "Returns string type for a message object of type 'ThrusterRaw"
  "hoverboard/ThrusterRaw")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ThrusterRaw>)))
  "Returns md5sum for a message object of type '<ThrusterRaw>"
  "d5ca7d3885d46960c93d211bf90a9632")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ThrusterRaw)))
  "Returns md5sum for a message object of type 'ThrusterRaw"
  "d5ca7d3885d46960c93d211bf90a9632")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ThrusterRaw>)))
  "Returns full string definition for message of type '<ThrusterRaw>"
  (cl:format cl:nil "Header header~%~%#Setpoints for the thrusters 0 off, 255 max~%uint8 thruster1~%uint8 thruster2~%uint8 thruster3~%uint8 thruster4~%uint8 thruster5~%uint8 thruster6~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ThrusterRaw)))
  "Returns full string definition for message of type 'ThrusterRaw"
  (cl:format cl:nil "Header header~%~%#Setpoints for the thrusters 0 off, 255 max~%uint8 thruster1~%uint8 thruster2~%uint8 thruster3~%uint8 thruster4~%uint8 thruster5~%uint8 thruster6~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ThrusterRaw>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ThrusterRaw>))
  "Converts a ROS message object to a list"
  (cl:list 'ThrusterRaw
    (cl:cons ':header (header msg))
    (cl:cons ':thruster1 (thruster1 msg))
    (cl:cons ':thruster2 (thruster2 msg))
    (cl:cons ':thruster3 (thruster3 msg))
    (cl:cons ':thruster4 (thruster4 msg))
    (cl:cons ':thruster5 (thruster5 msg))
    (cl:cons ':thruster6 (thruster6 msg))
))
