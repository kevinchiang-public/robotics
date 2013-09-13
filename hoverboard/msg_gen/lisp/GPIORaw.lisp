; Auto-generated. Do not edit!


(cl:in-package hoverboard-msg)


;//! \htmlinclude GPIORaw.msg.html

(cl:defclass <GPIORaw> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (gpio33_0
    :reader gpio33_0
    :initarg :gpio33_0
    :type cl:fixnum
    :initform 0)
   (gpio33_1
    :reader gpio33_1
    :initarg :gpio33_1
    :type cl:fixnum
    :initform 0)
   (gpio33_2
    :reader gpio33_2
    :initarg :gpio33_2
    :type cl:fixnum
    :initform 0)
   (gpio33_3
    :reader gpio33_3
    :initarg :gpio33_3
    :type cl:fixnum
    :initform 0)
   (gpio33_4
    :reader gpio33_4
    :initarg :gpio33_4
    :type cl:fixnum
    :initform 0)
   (gpio33_5
    :reader gpio33_5
    :initarg :gpio33_5
    :type cl:fixnum
    :initform 0)
   (gpio5_0
    :reader gpio5_0
    :initarg :gpio5_0
    :type cl:fixnum
    :initform 0)
   (gpio5_1
    :reader gpio5_1
    :initarg :gpio5_1
    :type cl:fixnum
    :initform 0)
   (gpio5_2
    :reader gpio5_2
    :initarg :gpio5_2
    :type cl:fixnum
    :initform 0)
   (gpio5_3
    :reader gpio5_3
    :initarg :gpio5_3
    :type cl:fixnum
    :initform 0)
   (gpio5_4
    :reader gpio5_4
    :initarg :gpio5_4
    :type cl:fixnum
    :initform 0)
   (gpio5_5
    :reader gpio5_5
    :initarg :gpio5_5
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GPIORaw (<GPIORaw>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GPIORaw>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GPIORaw)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hoverboard-msg:<GPIORaw> is deprecated: use hoverboard-msg:GPIORaw instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <GPIORaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hoverboard-msg:header-val is deprecated.  Use hoverboard-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'gpio33_0-val :lambda-list '(m))
(cl:defmethod gpio33_0-val ((m <GPIORaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hoverboard-msg:gpio33_0-val is deprecated.  Use hoverboard-msg:gpio33_0 instead.")
  (gpio33_0 m))

(cl:ensure-generic-function 'gpio33_1-val :lambda-list '(m))
(cl:defmethod gpio33_1-val ((m <GPIORaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hoverboard-msg:gpio33_1-val is deprecated.  Use hoverboard-msg:gpio33_1 instead.")
  (gpio33_1 m))

(cl:ensure-generic-function 'gpio33_2-val :lambda-list '(m))
(cl:defmethod gpio33_2-val ((m <GPIORaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hoverboard-msg:gpio33_2-val is deprecated.  Use hoverboard-msg:gpio33_2 instead.")
  (gpio33_2 m))

(cl:ensure-generic-function 'gpio33_3-val :lambda-list '(m))
(cl:defmethod gpio33_3-val ((m <GPIORaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hoverboard-msg:gpio33_3-val is deprecated.  Use hoverboard-msg:gpio33_3 instead.")
  (gpio33_3 m))

(cl:ensure-generic-function 'gpio33_4-val :lambda-list '(m))
(cl:defmethod gpio33_4-val ((m <GPIORaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hoverboard-msg:gpio33_4-val is deprecated.  Use hoverboard-msg:gpio33_4 instead.")
  (gpio33_4 m))

(cl:ensure-generic-function 'gpio33_5-val :lambda-list '(m))
(cl:defmethod gpio33_5-val ((m <GPIORaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hoverboard-msg:gpio33_5-val is deprecated.  Use hoverboard-msg:gpio33_5 instead.")
  (gpio33_5 m))

(cl:ensure-generic-function 'gpio5_0-val :lambda-list '(m))
(cl:defmethod gpio5_0-val ((m <GPIORaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hoverboard-msg:gpio5_0-val is deprecated.  Use hoverboard-msg:gpio5_0 instead.")
  (gpio5_0 m))

(cl:ensure-generic-function 'gpio5_1-val :lambda-list '(m))
(cl:defmethod gpio5_1-val ((m <GPIORaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hoverboard-msg:gpio5_1-val is deprecated.  Use hoverboard-msg:gpio5_1 instead.")
  (gpio5_1 m))

(cl:ensure-generic-function 'gpio5_2-val :lambda-list '(m))
(cl:defmethod gpio5_2-val ((m <GPIORaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hoverboard-msg:gpio5_2-val is deprecated.  Use hoverboard-msg:gpio5_2 instead.")
  (gpio5_2 m))

(cl:ensure-generic-function 'gpio5_3-val :lambda-list '(m))
(cl:defmethod gpio5_3-val ((m <GPIORaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hoverboard-msg:gpio5_3-val is deprecated.  Use hoverboard-msg:gpio5_3 instead.")
  (gpio5_3 m))

(cl:ensure-generic-function 'gpio5_4-val :lambda-list '(m))
(cl:defmethod gpio5_4-val ((m <GPIORaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hoverboard-msg:gpio5_4-val is deprecated.  Use hoverboard-msg:gpio5_4 instead.")
  (gpio5_4 m))

(cl:ensure-generic-function 'gpio5_5-val :lambda-list '(m))
(cl:defmethod gpio5_5-val ((m <GPIORaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hoverboard-msg:gpio5_5-val is deprecated.  Use hoverboard-msg:gpio5_5 instead.")
  (gpio5_5 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GPIORaw>) ostream)
  "Serializes a message object of type '<GPIORaw>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'gpio33_0)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'gpio33_1)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'gpio33_2)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'gpio33_3)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'gpio33_4)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'gpio33_5)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'gpio5_0)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'gpio5_1)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'gpio5_2)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'gpio5_3)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'gpio5_4)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'gpio5_5)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GPIORaw>) istream)
  "Deserializes a message object of type '<GPIORaw>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'gpio33_0) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'gpio33_1) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'gpio33_2) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'gpio33_3) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'gpio33_4) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'gpio33_5) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'gpio5_0) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'gpio5_1) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'gpio5_2) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'gpio5_3) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'gpio5_4) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'gpio5_5) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GPIORaw>)))
  "Returns string type for a message object of type '<GPIORaw>"
  "hoverboard/GPIORaw")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GPIORaw)))
  "Returns string type for a message object of type 'GPIORaw"
  "hoverboard/GPIORaw")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GPIORaw>)))
  "Returns md5sum for a message object of type '<GPIORaw>"
  "69e3b5824711a9cb06bff3f258b81857")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GPIORaw)))
  "Returns md5sum for a message object of type 'GPIORaw"
  "69e3b5824711a9cb06bff3f258b81857")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GPIORaw>)))
  "Returns full string definition for message of type '<GPIORaw>"
  (cl:format cl:nil "Header header~%~%# GPIO values for the 3.3V processor.  -1 indicates that~%# It has not been read, 0 if it is at a logic low level and~%# 1 for a logic high level~%int8 gpio33_0~%int8 gpio33_1~%int8 gpio33_2~%int8 gpio33_3~%int8 gpio33_4~%int8 gpio33_5~%~%# GPIO values for the 5.0V processor.  -1 indicates that~%# It has not been read, 0 if it is at a logic low level and~%# 1 for a logic high level~%int8 gpio5_0~%int8 gpio5_1~%int8 gpio5_2~%int8 gpio5_3~%int8 gpio5_4~%int8 gpio5_5~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GPIORaw)))
  "Returns full string definition for message of type 'GPIORaw"
  (cl:format cl:nil "Header header~%~%# GPIO values for the 3.3V processor.  -1 indicates that~%# It has not been read, 0 if it is at a logic low level and~%# 1 for a logic high level~%int8 gpio33_0~%int8 gpio33_1~%int8 gpio33_2~%int8 gpio33_3~%int8 gpio33_4~%int8 gpio33_5~%~%# GPIO values for the 5.0V processor.  -1 indicates that~%# It has not been read, 0 if it is at a logic low level and~%# 1 for a logic high level~%int8 gpio5_0~%int8 gpio5_1~%int8 gpio5_2~%int8 gpio5_3~%int8 gpio5_4~%int8 gpio5_5~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GPIORaw>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GPIORaw>))
  "Converts a ROS message object to a list"
  (cl:list 'GPIORaw
    (cl:cons ':header (header msg))
    (cl:cons ':gpio33_0 (gpio33_0 msg))
    (cl:cons ':gpio33_1 (gpio33_1 msg))
    (cl:cons ':gpio33_2 (gpio33_2 msg))
    (cl:cons ':gpio33_3 (gpio33_3 msg))
    (cl:cons ':gpio33_4 (gpio33_4 msg))
    (cl:cons ':gpio33_5 (gpio33_5 msg))
    (cl:cons ':gpio5_0 (gpio5_0 msg))
    (cl:cons ':gpio5_1 (gpio5_1 msg))
    (cl:cons ':gpio5_2 (gpio5_2 msg))
    (cl:cons ':gpio5_3 (gpio5_3 msg))
    (cl:cons ':gpio5_4 (gpio5_4 msg))
    (cl:cons ':gpio5_5 (gpio5_5 msg))
))
