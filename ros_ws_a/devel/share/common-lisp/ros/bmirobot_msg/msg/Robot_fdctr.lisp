; Auto-generated. Do not edit!


(cl:in-package bmirobot_msg-msg)


;//! \htmlinclude Robot_fdctr.msg.html

(cl:defclass <Robot_fdctr> (roslisp-msg-protocol:ros-message)
  ((mt_mode
    :reader mt_mode
    :initarg :mt_mode
    :type (cl:vector cl:integer)
   :initform (cl:make-array 9 :element-type 'cl:integer :initial-element 0))
   (mt_Cpst
    :reader mt_Cpst
    :initarg :mt_Cpst
    :type (cl:vector cl:integer)
   :initform (cl:make-array 9 :element-type 'cl:integer :initial-element 0))
   (mt_Cspd
    :reader mt_Cspd
    :initarg :mt_Cspd
    :type (cl:vector cl:integer)
   :initform (cl:make-array 9 :element-type 'cl:integer :initial-element 0))
   (mt_incrt
    :reader mt_incrt
    :initarg :mt_incrt
    :type (cl:vector cl:integer)
   :initform (cl:make-array 9 :element-type 'cl:integer :initial-element 0))
   (mt_PWMduty
    :reader mt_PWMduty
    :initarg :mt_PWMduty
    :type (cl:vector cl:integer)
   :initform (cl:make-array 9 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass Robot_fdctr (<Robot_fdctr>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Robot_fdctr>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Robot_fdctr)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bmirobot_msg-msg:<Robot_fdctr> is deprecated: use bmirobot_msg-msg:Robot_fdctr instead.")))

(cl:ensure-generic-function 'mt_mode-val :lambda-list '(m))
(cl:defmethod mt_mode-val ((m <Robot_fdctr>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bmirobot_msg-msg:mt_mode-val is deprecated.  Use bmirobot_msg-msg:mt_mode instead.")
  (mt_mode m))

(cl:ensure-generic-function 'mt_Cpst-val :lambda-list '(m))
(cl:defmethod mt_Cpst-val ((m <Robot_fdctr>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bmirobot_msg-msg:mt_Cpst-val is deprecated.  Use bmirobot_msg-msg:mt_Cpst instead.")
  (mt_Cpst m))

(cl:ensure-generic-function 'mt_Cspd-val :lambda-list '(m))
(cl:defmethod mt_Cspd-val ((m <Robot_fdctr>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bmirobot_msg-msg:mt_Cspd-val is deprecated.  Use bmirobot_msg-msg:mt_Cspd instead.")
  (mt_Cspd m))

(cl:ensure-generic-function 'mt_incrt-val :lambda-list '(m))
(cl:defmethod mt_incrt-val ((m <Robot_fdctr>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bmirobot_msg-msg:mt_incrt-val is deprecated.  Use bmirobot_msg-msg:mt_incrt instead.")
  (mt_incrt m))

(cl:ensure-generic-function 'mt_PWMduty-val :lambda-list '(m))
(cl:defmethod mt_PWMduty-val ((m <Robot_fdctr>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bmirobot_msg-msg:mt_PWMduty-val is deprecated.  Use bmirobot_msg-msg:mt_PWMduty instead.")
  (mt_PWMduty m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Robot_fdctr>) ostream)
  "Serializes a message object of type '<Robot_fdctr>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'mt_mode))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'mt_Cpst))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'mt_Cspd))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'mt_incrt))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'mt_PWMduty))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Robot_fdctr>) istream)
  "Deserializes a message object of type '<Robot_fdctr>"
  (cl:setf (cl:slot-value msg 'mt_mode) (cl:make-array 9))
  (cl:let ((vals (cl:slot-value msg 'mt_mode)))
    (cl:dotimes (i 9)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))))
  (cl:setf (cl:slot-value msg 'mt_Cpst) (cl:make-array 9))
  (cl:let ((vals (cl:slot-value msg 'mt_Cpst)))
    (cl:dotimes (i 9)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))))
  (cl:setf (cl:slot-value msg 'mt_Cspd) (cl:make-array 9))
  (cl:let ((vals (cl:slot-value msg 'mt_Cspd)))
    (cl:dotimes (i 9)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))))
  (cl:setf (cl:slot-value msg 'mt_incrt) (cl:make-array 9))
  (cl:let ((vals (cl:slot-value msg 'mt_incrt)))
    (cl:dotimes (i 9)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))))
  (cl:setf (cl:slot-value msg 'mt_PWMduty) (cl:make-array 9))
  (cl:let ((vals (cl:slot-value msg 'mt_PWMduty)))
    (cl:dotimes (i 9)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Robot_fdctr>)))
  "Returns string type for a message object of type '<Robot_fdctr>"
  "bmirobot_msg/Robot_fdctr")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Robot_fdctr)))
  "Returns string type for a message object of type 'Robot_fdctr"
  "bmirobot_msg/Robot_fdctr")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Robot_fdctr>)))
  "Returns md5sum for a message object of type '<Robot_fdctr>"
  "3195cf007256e8ddfd66fc62f56f5233")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Robot_fdctr)))
  "Returns md5sum for a message object of type 'Robot_fdctr"
  "3195cf007256e8ddfd66fc62f56f5233")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Robot_fdctr>)))
  "Returns full string definition for message of type '<Robot_fdctr>"
  (cl:format cl:nil "int32[9] mt_mode~%int32[9] mt_Cpst~%int32[9] mt_Cspd~%int32[9] mt_incrt~%int32[9] mt_PWMduty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Robot_fdctr)))
  "Returns full string definition for message of type 'Robot_fdctr"
  (cl:format cl:nil "int32[9] mt_mode~%int32[9] mt_Cpst~%int32[9] mt_Cspd~%int32[9] mt_incrt~%int32[9] mt_PWMduty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Robot_fdctr>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'mt_mode) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'mt_Cpst) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'mt_Cspd) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'mt_incrt) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'mt_PWMduty) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Robot_fdctr>))
  "Converts a ROS message object to a list"
  (cl:list 'Robot_fdctr
    (cl:cons ':mt_mode (mt_mode msg))
    (cl:cons ':mt_Cpst (mt_Cpst msg))
    (cl:cons ':mt_Cspd (mt_Cspd msg))
    (cl:cons ':mt_incrt (mt_incrt msg))
    (cl:cons ':mt_PWMduty (mt_PWMduty msg))
))
