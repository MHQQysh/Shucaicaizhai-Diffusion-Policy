; Auto-generated. Do not edit!


(cl:in-package bmirobot_msg-msg)


;//! \htmlinclude Robot_distance.msg.html

(cl:defclass <Robot_distance> (roslisp-msg-protocol:ros-message)
  ((proximity
    :reader proximity
    :initarg :proximity
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 25 :element-type 'cl:fixnum :initial-element 0))
   (realdistance
    :reader realdistance
    :initarg :realdistance
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 25 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass Robot_distance (<Robot_distance>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Robot_distance>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Robot_distance)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bmirobot_msg-msg:<Robot_distance> is deprecated: use bmirobot_msg-msg:Robot_distance instead.")))

(cl:ensure-generic-function 'proximity-val :lambda-list '(m))
(cl:defmethod proximity-val ((m <Robot_distance>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bmirobot_msg-msg:proximity-val is deprecated.  Use bmirobot_msg-msg:proximity instead.")
  (proximity m))

(cl:ensure-generic-function 'realdistance-val :lambda-list '(m))
(cl:defmethod realdistance-val ((m <Robot_distance>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bmirobot_msg-msg:realdistance-val is deprecated.  Use bmirobot_msg-msg:realdistance instead.")
  (realdistance m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Robot_distance>) ostream)
  "Serializes a message object of type '<Robot_distance>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'proximity))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'realdistance))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Robot_distance>) istream)
  "Deserializes a message object of type '<Robot_distance>"
  (cl:setf (cl:slot-value msg 'proximity) (cl:make-array 25))
  (cl:let ((vals (cl:slot-value msg 'proximity)))
    (cl:dotimes (i 25)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))))
  (cl:setf (cl:slot-value msg 'realdistance) (cl:make-array 25))
  (cl:let ((vals (cl:slot-value msg 'realdistance)))
    (cl:dotimes (i 25)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Robot_distance>)))
  "Returns string type for a message object of type '<Robot_distance>"
  "bmirobot_msg/Robot_distance")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Robot_distance)))
  "Returns string type for a message object of type 'Robot_distance"
  "bmirobot_msg/Robot_distance")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Robot_distance>)))
  "Returns md5sum for a message object of type '<Robot_distance>"
  "68c6629711bc8cca4c5688c4b92123ab")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Robot_distance)))
  "Returns md5sum for a message object of type 'Robot_distance"
  "68c6629711bc8cca4c5688c4b92123ab")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Robot_distance>)))
  "Returns full string definition for message of type '<Robot_distance>"
  (cl:format cl:nil "uint16[25] proximity~%uint16[25] realdistance~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Robot_distance)))
  "Returns full string definition for message of type 'Robot_distance"
  (cl:format cl:nil "uint16[25] proximity~%uint16[25] realdistance~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Robot_distance>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'proximity) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'realdistance) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Robot_distance>))
  "Converts a ROS message object to a list"
  (cl:list 'Robot_distance
    (cl:cons ':proximity (proximity msg))
    (cl:cons ':realdistance (realdistance msg))
))
