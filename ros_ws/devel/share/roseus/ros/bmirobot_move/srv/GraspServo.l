;; Auto-generated. Do not edit!


(when (boundp 'bmirobot_move::GraspServo)
  (if (not (find-package "BMIROBOT_MOVE"))
    (make-package "BMIROBOT_MOVE"))
  (shadow 'GraspServo (find-package "BMIROBOT_MOVE")))
(unless (find-package "BMIROBOT_MOVE::GRASPSERVO")
  (make-package "BMIROBOT_MOVE::GRASPSERVO"))
(unless (find-package "BMIROBOT_MOVE::GRASPSERVOREQUEST")
  (make-package "BMIROBOT_MOVE::GRASPSERVOREQUEST"))
(unless (find-package "BMIROBOT_MOVE::GRASPSERVORESPONSE")
  (make-package "BMIROBOT_MOVE::GRASPSERVORESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass bmirobot_move::GraspServoRequest
  :super ros::object
  :slots (_object_pose ))

(defmethod bmirobot_move::GraspServoRequest
  (:init
   (&key
    ((:object_pose __object_pose) (instance geometry_msgs::Pose :init))
    )
   (send-super :init)
   (setq _object_pose __object_pose)
   self)
  (:object_pose
   (&rest __object_pose)
   (if (keywordp (car __object_pose))
       (send* _object_pose __object_pose)
     (progn
       (if __object_pose (setq _object_pose (car __object_pose)))
       _object_pose)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Pose _object_pose
    (send _object_pose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Pose _object_pose
       (send _object_pose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Pose _object_pose
     (send _object_pose :deserialize buf ptr-) (incf ptr- (send _object_pose :serialization-length))
   ;;
   self)
  )

(defclass bmirobot_move::GraspServoResponse
  :super ros::object
  :slots (_status ))

(defmethod bmirobot_move::GraspServoResponse
  (:init
   (&key
    ((:status __status) 0)
    )
   (send-super :init)
   (setq _status (round __status))
   self)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:serialization-length
   ()
   (+
    ;; int32 _status
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _status
       (write-long _status s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _status
     (setq _status (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass bmirobot_move::GraspServo
  :super ros::object
  :slots ())

(setf (get bmirobot_move::GraspServo :md5sum-) "37b88a2b91372a258e4573b0ed517a08")
(setf (get bmirobot_move::GraspServo :datatype-) "bmirobot_move/GraspServo")
(setf (get bmirobot_move::GraspServo :request) bmirobot_move::GraspServoRequest)
(setf (get bmirobot_move::GraspServo :response) bmirobot_move::GraspServoResponse)

(defmethod bmirobot_move::GraspServoRequest
  (:response () (instance bmirobot_move::GraspServoResponse :init)))

(setf (get bmirobot_move::GraspServoRequest :md5sum-) "37b88a2b91372a258e4573b0ed517a08")
(setf (get bmirobot_move::GraspServoRequest :datatype-) "bmirobot_move/GraspServoRequest")
(setf (get bmirobot_move::GraspServoRequest :definition-)
      "geometry_msgs/Pose object_pose

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
---
int32 status

")

(setf (get bmirobot_move::GraspServoResponse :md5sum-) "37b88a2b91372a258e4573b0ed517a08")
(setf (get bmirobot_move::GraspServoResponse :datatype-) "bmirobot_move/GraspServoResponse")
(setf (get bmirobot_move::GraspServoResponse :definition-)
      "geometry_msgs/Pose object_pose

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
---
int32 status

")



(provide :bmirobot_move/GraspServo "37b88a2b91372a258e4573b0ed517a08")


