; Auto-generated. Do not edit!


(cl:in-package bmirobot_msg-msg)


;//! \htmlinclude motorv2_ctr.msg.html

(cl:defclass <motorv2_ctr> (roslisp-msg-protocol:ros-message)
  ((mt_operation_mode
    :reader mt_operation_mode
    :initarg :mt_operation_mode
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 8 :element-type 'cl:fixnum :initial-element 0))
   (mt_position_goal
    :reader mt_position_goal
    :initarg :mt_position_goal
    :type (cl:vector cl:integer)
   :initform (cl:make-array 8 :element-type 'cl:integer :initial-element 0))
   (mt_speed_goal
    :reader mt_speed_goal
    :initarg :mt_speed_goal
    :type (cl:vector cl:integer)
   :initform (cl:make-array 8 :element-type 'cl:integer :initial-element 0))
   (mt_current_goal
    :reader mt_current_goal
    :initarg :mt_current_goal
    :type (cl:vector cl:integer)
   :initform (cl:make-array 8 :element-type 'cl:integer :initial-element 0))
   (mt_pwm_goal
    :reader mt_pwm_goal
    :initarg :mt_pwm_goal
    :type (cl:vector cl:integer)
   :initform (cl:make-array 8 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass motorv2_ctr (<motorv2_ctr>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <motorv2_ctr>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'motorv2_ctr)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bmirobot_msg-msg:<motorv2_ctr> is deprecated: use bmirobot_msg-msg:motorv2_ctr instead.")))

(cl:ensure-generic-function 'mt_operation_mode-val :lambda-list '(m))
(cl:defmethod mt_operation_mode-val ((m <motorv2_ctr>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bmirobot_msg-msg:mt_operation_mode-val is deprecated.  Use bmirobot_msg-msg:mt_operation_mode instead.")
  (mt_operation_mode m))

(cl:ensure-generic-function 'mt_position_goal-val :lambda-list '(m))
(cl:defmethod mt_position_goal-val ((m <motorv2_ctr>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bmirobot_msg-msg:mt_position_goal-val is deprecated.  Use bmirobot_msg-msg:mt_position_goal instead.")
  (mt_position_goal m))

(cl:ensure-generic-function 'mt_speed_goal-val :lambda-list '(m))
(cl:defmethod mt_speed_goal-val ((m <motorv2_ctr>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bmirobot_msg-msg:mt_speed_goal-val is deprecated.  Use bmirobot_msg-msg:mt_speed_goal instead.")
  (mt_speed_goal m))

(cl:ensure-generic-function 'mt_current_goal-val :lambda-list '(m))
(cl:defmethod mt_current_goal-val ((m <motorv2_ctr>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bmirobot_msg-msg:mt_current_goal-val is deprecated.  Use bmirobot_msg-msg:mt_current_goal instead.")
  (mt_current_goal m))

(cl:ensure-generic-function 'mt_pwm_goal-val :lambda-list '(m))
(cl:defmethod mt_pwm_goal-val ((m <motorv2_ctr>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bmirobot_msg-msg:mt_pwm_goal-val is deprecated.  Use bmirobot_msg-msg:mt_pwm_goal instead.")
  (mt_pwm_goal m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <motorv2_ctr>) ostream)
  "Serializes a message object of type '<motorv2_ctr>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    ))
   (cl:slot-value msg 'mt_operation_mode))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'mt_position_goal))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'mt_speed_goal))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'mt_current_goal))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'mt_pwm_goal))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <motorv2_ctr>) istream)
  "Deserializes a message object of type '<motorv2_ctr>"
  (cl:setf (cl:slot-value msg 'mt_operation_mode) (cl:make-array 8))
  (cl:let ((vals (cl:slot-value msg 'mt_operation_mode)))
    (cl:dotimes (i 8)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))))
  (cl:setf (cl:slot-value msg 'mt_position_goal) (cl:make-array 8))
  (cl:let ((vals (cl:slot-value msg 'mt_position_goal)))
    (cl:dotimes (i 8)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))))
  (cl:setf (cl:slot-value msg 'mt_speed_goal) (cl:make-array 8))
  (cl:let ((vals (cl:slot-value msg 'mt_speed_goal)))
    (cl:dotimes (i 8)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))))
  (cl:setf (cl:slot-value msg 'mt_current_goal) (cl:make-array 8))
  (cl:let ((vals (cl:slot-value msg 'mt_current_goal)))
    (cl:dotimes (i 8)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))))
  (cl:setf (cl:slot-value msg 'mt_pwm_goal) (cl:make-array 8))
  (cl:let ((vals (cl:slot-value msg 'mt_pwm_goal)))
    (cl:dotimes (i 8)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<motorv2_ctr>)))
  "Returns string type for a message object of type '<motorv2_ctr>"
  "bmirobot_msg/motorv2_ctr")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'motorv2_ctr)))
  "Returns string type for a message object of type 'motorv2_ctr"
  "bmirobot_msg/motorv2_ctr")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<motorv2_ctr>)))
  "Returns md5sum for a message object of type '<motorv2_ctr>"
  "28290c9afcbb7e417ea5e1d8b11ed1b4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'motorv2_ctr)))
  "Returns md5sum for a message object of type 'motorv2_ctr"
  "28290c9afcbb7e417ea5e1d8b11ed1b4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<motorv2_ctr>)))
  "Returns full string definition for message of type '<motorv2_ctr>"
  (cl:format cl:nil "int8[8] mt_operation_mode~%int32[8] mt_position_goal~%int32[8] mt_speed_goal~%int32[8] mt_current_goal~%int32[8] mt_pwm_goal~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'motorv2_ctr)))
  "Returns full string definition for message of type 'motorv2_ctr"
  (cl:format cl:nil "int8[8] mt_operation_mode~%int32[8] mt_position_goal~%int32[8] mt_speed_goal~%int32[8] mt_current_goal~%int32[8] mt_pwm_goal~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <motorv2_ctr>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'mt_operation_mode) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'mt_position_goal) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'mt_speed_goal) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'mt_current_goal) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'mt_pwm_goal) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <motorv2_ctr>))
  "Converts a ROS message object to a list"
  (cl:list 'motorv2_ctr
    (cl:cons ':mt_operation_mode (mt_operation_mode msg))
    (cl:cons ':mt_position_goal (mt_position_goal msg))
    (cl:cons ':mt_speed_goal (mt_speed_goal msg))
    (cl:cons ':mt_current_goal (mt_current_goal msg))
    (cl:cons ':mt_pwm_goal (mt_pwm_goal msg))
))
