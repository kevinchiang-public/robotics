; Auto-generated. Do not edit!


(cl:in-package hoverboard-msg)


;//! \htmlinclude ADCRaw.msg.html

(cl:defclass <ADCRaw> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (adc5_0
    :reader adc5_0
    :initarg :adc5_0
    :type cl:integer
    :initform 0)
   (adc5_1
    :reader adc5_1
    :initarg :adc5_1
    :type cl:integer
    :initform 0)
   (adc5_2
    :reader adc5_2
    :initarg :adc5_2
    :type cl:integer
    :initform 0)
   (adc5_3
    :reader adc5_3
    :initarg :adc5_3
    :type cl:integer
    :initform 0)
   (adc5_4
    :reader adc5_4
    :initarg :adc5_4
    :type cl:integer
    :initform 0)
   (adc5_5
    :reader adc5_5
    :initarg :adc5_5
    :type cl:integer
    :initform 0)
   (adc5_6
    :reader adc5_6
    :initarg :adc5_6
    :type cl:integer
    :initform 0)
   (adc5_7
    :reader adc5_7
    :initarg :adc5_7
    :type cl:integer
    :initform 0)
   (adc33_0
    :reader adc33_0
    :initarg :adc33_0
    :type cl:integer
    :initform 0)
   (adc33_1
    :reader adc33_1
    :initarg :adc33_1
    :type cl:integer
    :initform 0)
   (adc33_2
    :reader adc33_2
    :initarg :adc33_2
    :type cl:integer
    :initform 0)
   (adc33_3
    :reader adc33_3
    :initarg :adc33_3
    :type cl:integer
    :initform 0)
   (adc33_4
    :reader adc33_4
    :initarg :adc33_4
    :type cl:integer
    :initform 0)
   (adc33_5
    :reader adc33_5
    :initarg :adc33_5
    :type cl:integer
    :initform 0))
)

(cl:defclass ADCRaw (<ADCRaw>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ADCRaw>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ADCRaw)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hoverboard-msg:<ADCRaw> is deprecated: use hoverboard-msg:ADCRaw instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ADCRaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hoverboard-msg:header-val is deprecated.  Use hoverboard-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'adc5_0-val :lambda-list '(m))
(cl:defmethod adc5_0-val ((m <ADCRaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hoverboard-msg:adc5_0-val is deprecated.  Use hoverboard-msg:adc5_0 instead.")
  (adc5_0 m))

(cl:ensure-generic-function 'adc5_1-val :lambda-list '(m))
(cl:defmethod adc5_1-val ((m <ADCRaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hoverboard-msg:adc5_1-val is deprecated.  Use hoverboard-msg:adc5_1 instead.")
  (adc5_1 m))

(cl:ensure-generic-function 'adc5_2-val :lambda-list '(m))
(cl:defmethod adc5_2-val ((m <ADCRaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hoverboard-msg:adc5_2-val is deprecated.  Use hoverboard-msg:adc5_2 instead.")
  (adc5_2 m))

(cl:ensure-generic-function 'adc5_3-val :lambda-list '(m))
(cl:defmethod adc5_3-val ((m <ADCRaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hoverboard-msg:adc5_3-val is deprecated.  Use hoverboard-msg:adc5_3 instead.")
  (adc5_3 m))

(cl:ensure-generic-function 'adc5_4-val :lambda-list '(m))
(cl:defmethod adc5_4-val ((m <ADCRaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hoverboard-msg:adc5_4-val is deprecated.  Use hoverboard-msg:adc5_4 instead.")
  (adc5_4 m))

(cl:ensure-generic-function 'adc5_5-val :lambda-list '(m))
(cl:defmethod adc5_5-val ((m <ADCRaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hoverboard-msg:adc5_5-val is deprecated.  Use hoverboard-msg:adc5_5 instead.")
  (adc5_5 m))

(cl:ensure-generic-function 'adc5_6-val :lambda-list '(m))
(cl:defmethod adc5_6-val ((m <ADCRaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hoverboard-msg:adc5_6-val is deprecated.  Use hoverboard-msg:adc5_6 instead.")
  (adc5_6 m))

(cl:ensure-generic-function 'adc5_7-val :lambda-list '(m))
(cl:defmethod adc5_7-val ((m <ADCRaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hoverboard-msg:adc5_7-val is deprecated.  Use hoverboard-msg:adc5_7 instead.")
  (adc5_7 m))

(cl:ensure-generic-function 'adc33_0-val :lambda-list '(m))
(cl:defmethod adc33_0-val ((m <ADCRaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hoverboard-msg:adc33_0-val is deprecated.  Use hoverboard-msg:adc33_0 instead.")
  (adc33_0 m))

(cl:ensure-generic-function 'adc33_1-val :lambda-list '(m))
(cl:defmethod adc33_1-val ((m <ADCRaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hoverboard-msg:adc33_1-val is deprecated.  Use hoverboard-msg:adc33_1 instead.")
  (adc33_1 m))

(cl:ensure-generic-function 'adc33_2-val :lambda-list '(m))
(cl:defmethod adc33_2-val ((m <ADCRaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hoverboard-msg:adc33_2-val is deprecated.  Use hoverboard-msg:adc33_2 instead.")
  (adc33_2 m))

(cl:ensure-generic-function 'adc33_3-val :lambda-list '(m))
(cl:defmethod adc33_3-val ((m <ADCRaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hoverboard-msg:adc33_3-val is deprecated.  Use hoverboard-msg:adc33_3 instead.")
  (adc33_3 m))

(cl:ensure-generic-function 'adc33_4-val :lambda-list '(m))
(cl:defmethod adc33_4-val ((m <ADCRaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hoverboard-msg:adc33_4-val is deprecated.  Use hoverboard-msg:adc33_4 instead.")
  (adc33_4 m))

(cl:ensure-generic-function 'adc33_5-val :lambda-list '(m))
(cl:defmethod adc33_5-val ((m <ADCRaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hoverboard-msg:adc33_5-val is deprecated.  Use hoverboard-msg:adc33_5 instead.")
  (adc33_5 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ADCRaw>) ostream)
  "Serializes a message object of type '<ADCRaw>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'adc5_0)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'adc5_1)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'adc5_2)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'adc5_3)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'adc5_4)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'adc5_5)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'adc5_6)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'adc5_7)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'adc33_0)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'adc33_1)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'adc33_2)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'adc33_3)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'adc33_4)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'adc33_5)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ADCRaw>) istream)
  "Deserializes a message object of type '<ADCRaw>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'adc5_0) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'adc5_1) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'adc5_2) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'adc5_3) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'adc5_4) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'adc5_5) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'adc5_6) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'adc5_7) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'adc33_0) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'adc33_1) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'adc33_2) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'adc33_3) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'adc33_4) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'adc33_5) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ADCRaw>)))
  "Returns string type for a message object of type '<ADCRaw>"
  "hoverboard/ADCRaw")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ADCRaw)))
  "Returns string type for a message object of type 'ADCRaw"
  "hoverboard/ADCRaw")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ADCRaw>)))
  "Returns md5sum for a message object of type '<ADCRaw>"
  "f5368e6fbeebc50a59ba28f656dd3599")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ADCRaw)))
  "Returns md5sum for a message object of type 'ADCRaw"
  "f5368e6fbeebc50a59ba28f656dd3599")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ADCRaw>)))
  "Returns full string definition for message of type '<ADCRaw>"
  (cl:format cl:nil "Header header~%~%#All of the adc values for the 5V processor on the hoverboard.  The~%#values are in mV, with -1 indicating this value was not~%#received/populated.~%~%int32 adc5_0~%int32 adc5_1~%int32 adc5_2~%int32 adc5_3~%int32 adc5_4~%int32 adc5_5~%int32 adc5_6~%int32 adc5_7~%~%#All of the adc values for the 3.3V processor on the hoverboard.  The~%#values are in mV, with -1 indicating this value was not~%#received/populated.~%~%int32 adc33_0~%int32 adc33_1~%int32 adc33_2~%int32 adc33_3~%int32 adc33_4~%int32 adc33_5~%~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ADCRaw)))
  "Returns full string definition for message of type 'ADCRaw"
  (cl:format cl:nil "Header header~%~%#All of the adc values for the 5V processor on the hoverboard.  The~%#values are in mV, with -1 indicating this value was not~%#received/populated.~%~%int32 adc5_0~%int32 adc5_1~%int32 adc5_2~%int32 adc5_3~%int32 adc5_4~%int32 adc5_5~%int32 adc5_6~%int32 adc5_7~%~%#All of the adc values for the 3.3V processor on the hoverboard.  The~%#values are in mV, with -1 indicating this value was not~%#received/populated.~%~%int32 adc33_0~%int32 adc33_1~%int32 adc33_2~%int32 adc33_3~%int32 adc33_4~%int32 adc33_5~%~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ADCRaw>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ADCRaw>))
  "Converts a ROS message object to a list"
  (cl:list 'ADCRaw
    (cl:cons ':header (header msg))
    (cl:cons ':adc5_0 (adc5_0 msg))
    (cl:cons ':adc5_1 (adc5_1 msg))
    (cl:cons ':adc5_2 (adc5_2 msg))
    (cl:cons ':adc5_3 (adc5_3 msg))
    (cl:cons ':adc5_4 (adc5_4 msg))
    (cl:cons ':adc5_5 (adc5_5 msg))
    (cl:cons ':adc5_6 (adc5_6 msg))
    (cl:cons ':adc5_7 (adc5_7 msg))
    (cl:cons ':adc33_0 (adc33_0 msg))
    (cl:cons ':adc33_1 (adc33_1 msg))
    (cl:cons ':adc33_2 (adc33_2 msg))
    (cl:cons ':adc33_3 (adc33_3 msg))
    (cl:cons ':adc33_4 (adc33_4 msg))
    (cl:cons ':adc33_5 (adc33_5 msg))
))
