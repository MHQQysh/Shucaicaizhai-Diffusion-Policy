;; Auto-generated. Do not edit!


(when (boundp 'bmirobot_msg::Robot_distance)
  (if (not (find-package "BMIROBOT_MSG"))
    (make-package "BMIROBOT_MSG"))
  (shadow 'Robot_distance (find-package "BMIROBOT_MSG")))
(unless (find-package "BMIROBOT_MSG::ROBOT_DISTANCE")
  (make-package "BMIROBOT_MSG::ROBOT_DISTANCE"))

(in-package "ROS")
;;//! \htmlinclude Robot_distance.msg.html


(defclass bmirobot_msg::Robot_distance
  :super ros::object
  :slots (_proximity _realdistance ))

(defmethod bmirobot_msg::Robot_distance
  (:init
   (&key
    ((:proximity __proximity) (make-array 25 :initial-element 0 :element-type :integer))
    ((:realdistance __realdistance) (make-array 25 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _proximity __proximity)
   (setq _realdistance __realdistance)
   self)
  (:proximity
   (&optional __proximity)
   (if __proximity (setq _proximity __proximity)) _proximity)
  (:realdistance
   (&optional __realdistance)
   (if __realdistance (setq _realdistance __realdistance)) _realdistance)
  (:serialization-length
   ()
   (+
    ;; uint16[25] _proximity
    (* 2    25)
    ;; uint16[25] _realdistance
    (* 2    25)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint16[25] _proximity
     (dotimes (i 25)
       (write-word (elt _proximity i) s)
       )
     ;; uint16[25] _realdistance
     (dotimes (i 25)
       (write-word (elt _realdistance i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint16[25] _proximity
   (dotimes (i (length _proximity))
     (setf (elt _proximity i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;; uint16[25] _realdistance
   (dotimes (i (length _realdistance))
     (setf (elt _realdistance i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;;
   self)
  )

(setf (get bmirobot_msg::Robot_distance :md5sum-) "68c6629711bc8cca4c5688c4b92123ab")
(setf (get bmirobot_msg::Robot_distance :datatype-) "bmirobot_msg/Robot_distance")
(setf (get bmirobot_msg::Robot_distance :definition-)
      "uint16[25] proximity
uint16[25] realdistance

")



(provide :bmirobot_msg/Robot_distance "68c6629711bc8cca4c5688c4b92123ab")


