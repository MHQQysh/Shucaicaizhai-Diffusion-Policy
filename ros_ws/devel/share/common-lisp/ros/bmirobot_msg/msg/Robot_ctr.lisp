; Auto-generated. Do not edit!


(cl:in-package bmirobot_msg-msg)


;//! \htmlinclude Robot_ctr.msg.html

(cl:defclass <Robot_ctr> (roslisp-msg-protocol:ros-message)
  ((mtID
    :reader mtID
    :initarg :mtID
    :type (cl:vector cl:integer)
   :initform (cl:make-array 9 :element-type 'cl:integer :initial-element 0))
   (mtmode
    :reader mtmode
    :initarg :mtmode
    :type (cl:vector cl:integer)
   :initform (cl:make-array 9 :element-type 'cl:integer :initial-element 0))
   (mtpst
    :reader mtpst
    :initarg :mtpst
    :type (cl:vector cl:integer)
   :initform (cl:make-array 9 :element-type 'cl:integer :initial-element 0))
   (mtspd
    :reader mtspd
    :initarg :mtspd
    :type (cl:vector cl:integer)
   :initform (cl:make-array 9 :element-type 'cl:integer :initial-element 0))
   (mttq
    :reader mttq
    :initarg :mttq
    :type (cl:vector cl:integer)
   :initform (cl:make-array 9 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass Robot_ctr (<Robot_ctr>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Robot_ctr>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Robot_ctr)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bmirobot_msg-msg:<Robot_ctr> is deprecated: use bmirobot_msg-msg:Robot_ctr instead.")))

(cl:ensure-generic-function 'mtID-val :lambda-list '(m))
(cl:defmethod mtID-val ((m <Robot_ctr>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bmirobot_msg-msg:mtID-val is deprecated.  Use bmirobot_msg-msg:mtID instead.")
  (mtID m))

(cl:ensure-generic-function 'mtmode-val :lambda-list '(m))
(cl:defmethod mtmode-val ((m <Robot_ctr>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bmirobot_msg-msg:mtmode-val is deprecated.  Use bmirobot_msg-msg:mtmode instead.")
  (mtmode m))

(cl:ensure-generic-function 'mtpst-val :lambda-list '(m))
(cl:defmethod mtpst-val ((m <Robot_ctr>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bmirobot_msg-msg:mtpst-val is deprecated.  Use bmirobot_msg-msg:mtpst instead.")
  (mtpst m))

(cl:ensure-generic-function 'mtspd-val :lambda-list '(m))
(cl:defmethod mtspd-val ((m <Robot_ctr>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bmirobot_msg-msg:mtspd-val is deprecated.  Use bmirobot_msg-msg:mtspd instead.")
  (mtspd m))

(cl:ensure-generic-function 'mttq-val :lambda-list '(m))
(cl:defmethod mttq-val ((m <Robot_ctr>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bmirobot_msg-msg:mttq-val is deprecated.  Use bmirobot_msg-msg:mttq instead.")
  (mttq m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Robot_ctr>) ostream)
  "Serializes a message object of type '<Robot_ctr>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'mtID))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'mtmode))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'mtpst))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'mtspd))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'mttq))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Robot_ctr>) istream)
  "Deserializes a message object of type '<Robot_ctr>"
  (cl:setf (cl:slot-value msg 'mtID) (cl:make-array 9))
  (cl:let ((vals (cl:slot-value msg 'mtID)))
    (cl:dotimes (i 9)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))))
  (cl:setf (cl:slot-value msg 'mtmode) (cl:make-array 9))
  (cl:let ((vals (cl:slot-value msg 'mtmode)))
    (cl:dotimes (i 9)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))))
  (cl:setf (cl:slot-value msg 'mtpst) (cl:make-array 9))
  (cl:let ((vals (cl:slot-value msg 'mtpst)))
    (cl:dotimes (i 9)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))))
  (cl:setf (cl:slot-value msg 'mtspd) (cl:make-array 9))
  (cl:let ((vals (cl:slot-value msg 'mtspd)))
    (cl:dotimes (i 9)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))))
  (cl:setf (cl:slot-value msg 'mttq) (cl:make-array 9))
  (cl:let ((vals (cl:slot-value msg 'mttq)))
    (cl:dotimes (i 9)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Robot_ctr>)))
  "Returns string type for a message object of type '<Robot_ctr>"
  "bmirobot_msg/Robot_ctr")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Robot_ctr)))
  "Returns string type for a message object of type 'Robot_ctr"
  "bmirobot_msg/Robot_ctr")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Robot_ctr>)))
  "Returns md5sum for a message object of type '<Robot_ctr>"
  "0fc2a2db85d9265b43f59ed7bec2ae3c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Robot_ctr)))
  "Returns md5sum for a message object of type 'Robot_ctr"
  "0fc2a2db85d9265b43f59ed7bec2ae3c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Robot_ctr>)))
  "Returns full string definition for message of type '<Robot_ctr>"
  (cl:format cl:nil "int32[9] mtID~%int32[9] mtmode~%int32[9] mtpst~%int32[9] mtspd~%int32[9] mttq~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Robot_ctr)))
  "Returns full string definition for message of type 'Robot_ctr"
  (cl:format cl:nil "int32[9] mtID~%int32[9] mtmode~%int32[9] mtpst~%int32[9] mtspd~%int32[9] mttq~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Robot_ctr>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'mtID) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'mtmode) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'mtpst) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'mtspd) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'mttq) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Robot_ctr>))
  "Converts a ROS message object to a list"
  (cl:list 'Robot_ctr
    (cl:cons ':mtID (mtID msg))
    (cl:cons ':mtmode (mtmode msg))
    (cl:cons ':mtpst (mtpst msg))
    (cl:cons ':mtspd (mtspd msg))
    (cl:cons ':mttq (mttq msg))
))
