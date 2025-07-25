; Auto-generated. Do not edit!


(cl:in-package bmirobot_msg-msg)


;//! \htmlinclude Robot_mpu.msg.html

(cl:defclass <Robot_mpu> (roslisp-msg-protocol:ros-message)
  ((mpu_Ax
    :reader mpu_Ax
    :initarg :mpu_Ax
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 8 :element-type 'cl:fixnum :initial-element 0))
   (mpu_Ay
    :reader mpu_Ay
    :initarg :mpu_Ay
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 8 :element-type 'cl:fixnum :initial-element 0))
   (mpu_Az
    :reader mpu_Az
    :initarg :mpu_Az
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 8 :element-type 'cl:fixnum :initial-element 0))
   (mpu_Rx
    :reader mpu_Rx
    :initarg :mpu_Rx
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 8 :element-type 'cl:fixnum :initial-element 0))
   (mpu_Ry
    :reader mpu_Ry
    :initarg :mpu_Ry
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 8 :element-type 'cl:fixnum :initial-element 0))
   (mpu_Rz
    :reader mpu_Rz
    :initarg :mpu_Rz
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 8 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass Robot_mpu (<Robot_mpu>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Robot_mpu>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Robot_mpu)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bmirobot_msg-msg:<Robot_mpu> is deprecated: use bmirobot_msg-msg:Robot_mpu instead.")))

(cl:ensure-generic-function 'mpu_Ax-val :lambda-list '(m))
(cl:defmethod mpu_Ax-val ((m <Robot_mpu>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bmirobot_msg-msg:mpu_Ax-val is deprecated.  Use bmirobot_msg-msg:mpu_Ax instead.")
  (mpu_Ax m))

(cl:ensure-generic-function 'mpu_Ay-val :lambda-list '(m))
(cl:defmethod mpu_Ay-val ((m <Robot_mpu>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bmirobot_msg-msg:mpu_Ay-val is deprecated.  Use bmirobot_msg-msg:mpu_Ay instead.")
  (mpu_Ay m))

(cl:ensure-generic-function 'mpu_Az-val :lambda-list '(m))
(cl:defmethod mpu_Az-val ((m <Robot_mpu>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bmirobot_msg-msg:mpu_Az-val is deprecated.  Use bmirobot_msg-msg:mpu_Az instead.")
  (mpu_Az m))

(cl:ensure-generic-function 'mpu_Rx-val :lambda-list '(m))
(cl:defmethod mpu_Rx-val ((m <Robot_mpu>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bmirobot_msg-msg:mpu_Rx-val is deprecated.  Use bmirobot_msg-msg:mpu_Rx instead.")
  (mpu_Rx m))

(cl:ensure-generic-function 'mpu_Ry-val :lambda-list '(m))
(cl:defmethod mpu_Ry-val ((m <Robot_mpu>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bmirobot_msg-msg:mpu_Ry-val is deprecated.  Use bmirobot_msg-msg:mpu_Ry instead.")
  (mpu_Ry m))

(cl:ensure-generic-function 'mpu_Rz-val :lambda-list '(m))
(cl:defmethod mpu_Rz-val ((m <Robot_mpu>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bmirobot_msg-msg:mpu_Rz-val is deprecated.  Use bmirobot_msg-msg:mpu_Rz instead.")
  (mpu_Rz m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Robot_mpu>) ostream)
  "Serializes a message object of type '<Robot_mpu>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'mpu_Ax))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'mpu_Ay))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'mpu_Az))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'mpu_Rx))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'mpu_Ry))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'mpu_Rz))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Robot_mpu>) istream)
  "Deserializes a message object of type '<Robot_mpu>"
  (cl:setf (cl:slot-value msg 'mpu_Ax) (cl:make-array 8))
  (cl:let ((vals (cl:slot-value msg 'mpu_Ax)))
    (cl:dotimes (i 8)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))))
  (cl:setf (cl:slot-value msg 'mpu_Ay) (cl:make-array 8))
  (cl:let ((vals (cl:slot-value msg 'mpu_Ay)))
    (cl:dotimes (i 8)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))))
  (cl:setf (cl:slot-value msg 'mpu_Az) (cl:make-array 8))
  (cl:let ((vals (cl:slot-value msg 'mpu_Az)))
    (cl:dotimes (i 8)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))))
  (cl:setf (cl:slot-value msg 'mpu_Rx) (cl:make-array 8))
  (cl:let ((vals (cl:slot-value msg 'mpu_Rx)))
    (cl:dotimes (i 8)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))))
  (cl:setf (cl:slot-value msg 'mpu_Ry) (cl:make-array 8))
  (cl:let ((vals (cl:slot-value msg 'mpu_Ry)))
    (cl:dotimes (i 8)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))))
  (cl:setf (cl:slot-value msg 'mpu_Rz) (cl:make-array 8))
  (cl:let ((vals (cl:slot-value msg 'mpu_Rz)))
    (cl:dotimes (i 8)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Robot_mpu>)))
  "Returns string type for a message object of type '<Robot_mpu>"
  "bmirobot_msg/Robot_mpu")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Robot_mpu)))
  "Returns string type for a message object of type 'Robot_mpu"
  "bmirobot_msg/Robot_mpu")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Robot_mpu>)))
  "Returns md5sum for a message object of type '<Robot_mpu>"
  "742fca3c0a78013d1f45f4495e1ad202")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Robot_mpu)))
  "Returns md5sum for a message object of type 'Robot_mpu"
  "742fca3c0a78013d1f45f4495e1ad202")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Robot_mpu>)))
  "Returns full string definition for message of type '<Robot_mpu>"
  (cl:format cl:nil "int16[8] mpu_Ax~%int16[8] mpu_Ay~%int16[8] mpu_Az~%int16[8] mpu_Rx~%int16[8] mpu_Ry~%int16[8] mpu_Rz~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Robot_mpu)))
  "Returns full string definition for message of type 'Robot_mpu"
  (cl:format cl:nil "int16[8] mpu_Ax~%int16[8] mpu_Ay~%int16[8] mpu_Az~%int16[8] mpu_Rx~%int16[8] mpu_Ry~%int16[8] mpu_Rz~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Robot_mpu>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'mpu_Ax) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'mpu_Ay) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'mpu_Az) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'mpu_Rx) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'mpu_Ry) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'mpu_Rz) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Robot_mpu>))
  "Converts a ROS message object to a list"
  (cl:list 'Robot_mpu
    (cl:cons ':mpu_Ax (mpu_Ax msg))
    (cl:cons ':mpu_Ay (mpu_Ay msg))
    (cl:cons ':mpu_Az (mpu_Az msg))
    (cl:cons ':mpu_Rx (mpu_Rx msg))
    (cl:cons ':mpu_Ry (mpu_Ry msg))
    (cl:cons ':mpu_Rz (mpu_Rz msg))
))
