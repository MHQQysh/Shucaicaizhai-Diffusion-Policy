; Auto-generated. Do not edit!


(cl:in-package bmirobot_msg-msg)


;//! \htmlinclude Robot_jointfd.msg.html

(cl:defclass <Robot_jointfd> (roslisp-msg-protocol:ros-message)
  ((Joint_fdpst
    :reader Joint_fdpst
    :initarg :Joint_fdpst
    :type (cl:vector cl:float)
   :initform (cl:make-array 9 :element-type 'cl:float :initial-element 0.0))
   (Joint_fdspd
    :reader Joint_fdspd
    :initarg :Joint_fdspd
    :type (cl:vector cl:float)
   :initform (cl:make-array 9 :element-type 'cl:float :initial-element 0.0))
   (Joint_fdctr
    :reader Joint_fdctr
    :initarg :Joint_fdctr
    :type (cl:vector cl:float)
   :initform (cl:make-array 9 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass Robot_jointfd (<Robot_jointfd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Robot_jointfd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Robot_jointfd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bmirobot_msg-msg:<Robot_jointfd> is deprecated: use bmirobot_msg-msg:Robot_jointfd instead.")))

(cl:ensure-generic-function 'Joint_fdpst-val :lambda-list '(m))
(cl:defmethod Joint_fdpst-val ((m <Robot_jointfd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bmirobot_msg-msg:Joint_fdpst-val is deprecated.  Use bmirobot_msg-msg:Joint_fdpst instead.")
  (Joint_fdpst m))

(cl:ensure-generic-function 'Joint_fdspd-val :lambda-list '(m))
(cl:defmethod Joint_fdspd-val ((m <Robot_jointfd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bmirobot_msg-msg:Joint_fdspd-val is deprecated.  Use bmirobot_msg-msg:Joint_fdspd instead.")
  (Joint_fdspd m))

(cl:ensure-generic-function 'Joint_fdctr-val :lambda-list '(m))
(cl:defmethod Joint_fdctr-val ((m <Robot_jointfd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bmirobot_msg-msg:Joint_fdctr-val is deprecated.  Use bmirobot_msg-msg:Joint_fdctr instead.")
  (Joint_fdctr m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Robot_jointfd>) ostream)
  "Serializes a message object of type '<Robot_jointfd>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'Joint_fdpst))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'Joint_fdspd))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'Joint_fdctr))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Robot_jointfd>) istream)
  "Deserializes a message object of type '<Robot_jointfd>"
  (cl:setf (cl:slot-value msg 'Joint_fdpst) (cl:make-array 9))
  (cl:let ((vals (cl:slot-value msg 'Joint_fdpst)))
    (cl:dotimes (i 9)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'Joint_fdspd) (cl:make-array 9))
  (cl:let ((vals (cl:slot-value msg 'Joint_fdspd)))
    (cl:dotimes (i 9)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'Joint_fdctr) (cl:make-array 9))
  (cl:let ((vals (cl:slot-value msg 'Joint_fdctr)))
    (cl:dotimes (i 9)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Robot_jointfd>)))
  "Returns string type for a message object of type '<Robot_jointfd>"
  "bmirobot_msg/Robot_jointfd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Robot_jointfd)))
  "Returns string type for a message object of type 'Robot_jointfd"
  "bmirobot_msg/Robot_jointfd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Robot_jointfd>)))
  "Returns md5sum for a message object of type '<Robot_jointfd>"
  "8b60b3db716c9968ae69daf16554d81f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Robot_jointfd)))
  "Returns md5sum for a message object of type 'Robot_jointfd"
  "8b60b3db716c9968ae69daf16554d81f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Robot_jointfd>)))
  "Returns full string definition for message of type '<Robot_jointfd>"
  (cl:format cl:nil "float32[9] Joint_fdpst~%float32[9] Joint_fdspd~%float32[9] Joint_fdctr~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Robot_jointfd)))
  "Returns full string definition for message of type 'Robot_jointfd"
  (cl:format cl:nil "float32[9] Joint_fdpst~%float32[9] Joint_fdspd~%float32[9] Joint_fdctr~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Robot_jointfd>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'Joint_fdpst) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'Joint_fdspd) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'Joint_fdctr) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Robot_jointfd>))
  "Converts a ROS message object to a list"
  (cl:list 'Robot_jointfd
    (cl:cons ':Joint_fdpst (Joint_fdpst msg))
    (cl:cons ':Joint_fdspd (Joint_fdspd msg))
    (cl:cons ':Joint_fdctr (Joint_fdctr msg))
))
