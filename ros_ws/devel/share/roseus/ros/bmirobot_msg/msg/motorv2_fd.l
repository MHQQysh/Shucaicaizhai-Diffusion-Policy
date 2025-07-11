;; Auto-generated. Do not edit!


(when (boundp 'bmirobot_msg::motorv2_fd)
  (if (not (find-package "BMIROBOT_MSG"))
    (make-package "BMIROBOT_MSG"))
  (shadow 'motorv2_fd (find-package "BMIROBOT_MSG")))
(unless (find-package "BMIROBOT_MSG::MOTORV2_FD")
  (make-package "BMIROBOT_MSG::MOTORV2_FD"))

(in-package "ROS")
;;//! \htmlinclude motorv2_fd.msg.html


(defclass bmirobot_msg::motorv2_fd
  :super ros::object
  :slots (_mt_position _mt_speed _mt_pwm _mt_current ))

(defmethod bmirobot_msg::motorv2_fd
  (:init
   (&key
    ((:mt_position __mt_position) (make-array 8 :initial-element 0 :element-type :integer))
    ((:mt_speed __mt_speed) (make-array 8 :initial-element 0 :element-type :integer))
    ((:mt_pwm __mt_pwm) (make-array 8 :initial-element 0 :element-type :integer))
    ((:mt_current __mt_current) (make-array 8 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _mt_position __mt_position)
   (setq _mt_speed __mt_speed)
   (setq _mt_pwm __mt_pwm)
   (setq _mt_current __mt_current)
   self)
  (:mt_position
   (&optional __mt_position)
   (if __mt_position (setq _mt_position __mt_position)) _mt_position)
  (:mt_speed
   (&optional __mt_speed)
   (if __mt_speed (setq _mt_speed __mt_speed)) _mt_speed)
  (:mt_pwm
   (&optional __mt_pwm)
   (if __mt_pwm (setq _mt_pwm __mt_pwm)) _mt_pwm)
  (:mt_current
   (&optional __mt_current)
   (if __mt_current (setq _mt_current __mt_current)) _mt_current)
  (:serialization-length
   ()
   (+
    ;; int32[8] _mt_position
    (* 4    8)
    ;; int32[8] _mt_speed
    (* 4    8)
    ;; int16[8] _mt_pwm
    (* 2    8)
    ;; int16[8] _mt_current
    (* 2    8)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32[8] _mt_position
     (dotimes (i 8)
       (write-long (elt _mt_position i) s)
       )
     ;; int32[8] _mt_speed
     (dotimes (i 8)
       (write-long (elt _mt_speed i) s)
       )
     ;; int16[8] _mt_pwm
     (dotimes (i 8)
       (write-word (elt _mt_pwm i) s)
       )
     ;; int16[8] _mt_current
     (dotimes (i 8)
       (write-word (elt _mt_current i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32[8] _mt_position
   (dotimes (i (length _mt_position))
     (setf (elt _mt_position i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;; int32[8] _mt_speed
   (dotimes (i (length _mt_speed))
     (setf (elt _mt_speed i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;; int16[8] _mt_pwm
   (dotimes (i (length _mt_pwm))
     (setf (elt _mt_pwm i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;; int16[8] _mt_current
   (dotimes (i (length _mt_current))
     (setf (elt _mt_current i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;;
   self)
  )

(setf (get bmirobot_msg::motorv2_fd :md5sum-) "a5d41159e774c193e38fa281a354be65")
(setf (get bmirobot_msg::motorv2_fd :datatype-) "bmirobot_msg/motorv2_fd")
(setf (get bmirobot_msg::motorv2_fd :definition-)
      "int32[8] mt_position
int32[8] mt_speed
int16[8] mt_pwm
int16[8] mt_current
")



(provide :bmirobot_msg/motorv2_fd "a5d41159e774c193e38fa281a354be65")


