; Auto-generated. Do not edit!


(cl:in-package bmirobot_msg-msg)


;//! \htmlinclude motorv2_fd.msg.html

(cl:defclass <motorv2_fd> (roslisp-msg-protocol:ros-message)
  ((mt_position
    :reader mt_position
    :initarg :mt_position
    :type (cl:vector cl:integer)
   :initform (cl:make-array 8 :element-type 'cl:integer :initial-element 0))
   (mt_speed
    :reader mt_speed
    :initarg :mt_speed
    :type (cl:vector cl:integer)
   :initform (cl:make-array 8 :element-type 'cl:integer :initial-element 0))
   (mt_pwm
    :reader mt_pwm
    :initarg :mt_pwm
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 8 :element-type 'cl:fixnum :initial-element 0))
   (mt_current
    :reader mt_current
    :initarg :mt_current
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 8 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass motorv2_fd (<motorv2_fd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <motorv2_fd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'motorv2_fd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bmirobot_msg-msg:<motorv2_fd> is deprecated: use bmirobot_msg-msg:motorv2_fd instead.")))

(cl:ensure-generic-function 'mt_position-val :lambda-list '(m))
(cl:defmethod mt_position-val ((m <motorv2_fd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bmirobot_msg-msg:mt_position-val is deprecated.  Use bmirobot_msg-msg:mt_position instead.")
  (mt_position m))

(cl:ensure-generic-function 'mt_speed-val :lambda-list '(m))
(cl:defmethod mt_speed-val ((m <motorv2_fd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bmirobot_msg-msg:mt_speed-val is deprecated.  Use bmirobot_msg-msg:mt_speed instead.")
  (mt_speed m))

(cl:ensure-generic-function 'mt_pwm-val :lambda-list '(m))
(cl:defmethod mt_pwm-val ((m <motorv2_fd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bmirobot_msg-msg:mt_pwm-val is deprecated.  Use bmirobot_msg-msg:mt_pwm instead.")
  (mt_pwm m))

(cl:ensure-generic-function 'mt_current-val :lambda-list '(m))
(cl:defmethod mt_current-val ((m <motorv2_fd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bmirobot_msg-msg:mt_current-val is deprecated.  Use bmirobot_msg-msg:mt_current instead.")
  (mt_current m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <motorv2_fd>) ostream)
  "Serializes a message object of type '<motorv2_fd>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'mt_position))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'mt_speed))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'mt_pwm))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'mt_current))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <motorv2_fd>) istream)
  "Deserializes a message object of type '<motorv2_fd>"
  (cl:setf (cl:slot-value msg 'mt_position) (cl:make-array 8))
  (cl:let ((vals (cl:slot-value msg 'mt_position)))
    (cl:dotimes (i 8)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))))
  (cl:setf (cl:slot-value msg 'mt_speed) (cl:make-array 8))
  (cl:let ((vals (cl:slot-value msg 'mt_speed)))
    (cl:dotimes (i 8)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))))
  (cl:setf (cl:slot-value msg 'mt_pwm) (cl:make-array 8))
  (cl:let ((vals (cl:slot-value msg 'mt_pwm)))
    (cl:dotimes (i 8)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))))
  (cl:setf (cl:slot-value msg 'mt_current) (cl:make-array 8))
  (cl:let ((vals (cl:slot-value msg 'mt_current)))
    (cl:dotimes (i 8)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<motorv2_fd>)))
  "Returns string type for a message object of type '<motorv2_fd>"
  "bmirobot_msg/motorv2_fd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'motorv2_fd)))
  "Returns string type for a message object of type 'motorv2_fd"
  "bmirobot_msg/motorv2_fd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<motorv2_fd>)))
  "Returns md5sum for a message object of type '<motorv2_fd>"
  "a5d41159e774c193e38fa281a354be65")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'motorv2_fd)))
  "Returns md5sum for a message object of type 'motorv2_fd"
  "a5d41159e774c193e38fa281a354be65")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<motorv2_fd>)))
  "Returns full string definition for message of type '<motorv2_fd>"
  (cl:format cl:nil "int32[8] mt_position~%int32[8] mt_speed~%int16[8] mt_pwm~%int16[8] mt_current~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'motorv2_fd)))
  "Returns full string definition for message of type 'motorv2_fd"
  (cl:format cl:nil "int32[8] mt_position~%int32[8] mt_speed~%int16[8] mt_pwm~%int16[8] mt_current~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <motorv2_fd>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'mt_position) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'mt_speed) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'mt_pwm) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'mt_current) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <motorv2_fd>))
  "Converts a ROS message object to a list"
  (cl:list 'motorv2_fd
    (cl:cons ':mt_position (mt_position msg))
    (cl:cons ':mt_speed (mt_speed msg))
    (cl:cons ':mt_pwm (mt_pwm msg))
    (cl:cons ':mt_current (mt_current msg))
))
