; Auto-generated. Do not edit!


(cl:in-package bmirobot_move-srv)


;//! \htmlinclude GraspServo-request.msg.html

(cl:defclass <GraspServo-request> (roslisp-msg-protocol:ros-message)
  ((object_pose
    :reader object_pose
    :initarg :object_pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose)))
)

(cl:defclass GraspServo-request (<GraspServo-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GraspServo-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GraspServo-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bmirobot_move-srv:<GraspServo-request> is deprecated: use bmirobot_move-srv:GraspServo-request instead.")))

(cl:ensure-generic-function 'object_pose-val :lambda-list '(m))
(cl:defmethod object_pose-val ((m <GraspServo-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bmirobot_move-srv:object_pose-val is deprecated.  Use bmirobot_move-srv:object_pose instead.")
  (object_pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GraspServo-request>) ostream)
  "Serializes a message object of type '<GraspServo-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'object_pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GraspServo-request>) istream)
  "Deserializes a message object of type '<GraspServo-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'object_pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GraspServo-request>)))
  "Returns string type for a service object of type '<GraspServo-request>"
  "bmirobot_move/GraspServoRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GraspServo-request)))
  "Returns string type for a service object of type 'GraspServo-request"
  "bmirobot_move/GraspServoRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GraspServo-request>)))
  "Returns md5sum for a message object of type '<GraspServo-request>"
  "37b88a2b91372a258e4573b0ed517a08")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GraspServo-request)))
  "Returns md5sum for a message object of type 'GraspServo-request"
  "37b88a2b91372a258e4573b0ed517a08")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GraspServo-request>)))
  "Returns full string definition for message of type '<GraspServo-request>"
  (cl:format cl:nil "geometry_msgs/Pose object_pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GraspServo-request)))
  "Returns full string definition for message of type 'GraspServo-request"
  (cl:format cl:nil "geometry_msgs/Pose object_pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GraspServo-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'object_pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GraspServo-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GraspServo-request
    (cl:cons ':object_pose (object_pose msg))
))
;//! \htmlinclude GraspServo-response.msg.html

(cl:defclass <GraspServo-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:integer
    :initform 0))
)

(cl:defclass GraspServo-response (<GraspServo-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GraspServo-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GraspServo-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bmirobot_move-srv:<GraspServo-response> is deprecated: use bmirobot_move-srv:GraspServo-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <GraspServo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bmirobot_move-srv:status-val is deprecated.  Use bmirobot_move-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GraspServo-response>) ostream)
  "Serializes a message object of type '<GraspServo-response>"
  (cl:let* ((signed (cl:slot-value msg 'status)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GraspServo-response>) istream)
  "Deserializes a message object of type '<GraspServo-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GraspServo-response>)))
  "Returns string type for a service object of type '<GraspServo-response>"
  "bmirobot_move/GraspServoResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GraspServo-response)))
  "Returns string type for a service object of type 'GraspServo-response"
  "bmirobot_move/GraspServoResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GraspServo-response>)))
  "Returns md5sum for a message object of type '<GraspServo-response>"
  "37b88a2b91372a258e4573b0ed517a08")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GraspServo-response)))
  "Returns md5sum for a message object of type 'GraspServo-response"
  "37b88a2b91372a258e4573b0ed517a08")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GraspServo-response>)))
  "Returns full string definition for message of type '<GraspServo-response>"
  (cl:format cl:nil "int32 status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GraspServo-response)))
  "Returns full string definition for message of type 'GraspServo-response"
  (cl:format cl:nil "int32 status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GraspServo-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GraspServo-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GraspServo-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GraspServo)))
  'GraspServo-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GraspServo)))
  'GraspServo-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GraspServo)))
  "Returns string type for a service object of type '<GraspServo>"
  "bmirobot_move/GraspServo")