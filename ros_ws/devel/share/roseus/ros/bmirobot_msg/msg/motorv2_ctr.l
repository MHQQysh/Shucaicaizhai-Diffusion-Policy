;; Auto-generated. Do not edit!


(when (boundp 'bmirobot_msg::motorv2_ctr)
  (if (not (find-package "BMIROBOT_MSG"))
    (make-package "BMIROBOT_MSG"))
  (shadow 'motorv2_ctr (find-package "BMIROBOT_MSG")))
(unless (find-package "BMIROBOT_MSG::MOTORV2_CTR")
  (make-package "BMIROBOT_MSG::MOTORV2_CTR"))

(in-package "ROS")
;;//! \htmlinclude motorv2_ctr.msg.html


(defclass bmirobot_msg::motorv2_ctr
  :super ros::object
  :slots (_mt_operation_mode _mt_position_goal _mt_speed_goal _mt_current_goal _mt_pwm_goal ))

(defmethod bmirobot_msg::motorv2_ctr
  (:init
   (&key
    ((:mt_operation_mode __mt_operation_mode) (make-array 8 :initial-element 0 :element-type :integer))
    ((:mt_position_goal __mt_position_goal) (make-array 8 :initial-element 0 :element-type :integer))
    ((:mt_speed_goal __mt_speed_goal) (make-array 8 :initial-element 0 :element-type :integer))
    ((:mt_current_goal __mt_current_goal) (make-array 8 :initial-element 0 :element-type :integer))
    ((:mt_pwm_goal __mt_pwm_goal) (make-array 8 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _mt_operation_mode __mt_operation_mode)
   (setq _mt_position_goal __mt_position_goal)
   (setq _mt_speed_goal __mt_speed_goal)
   (setq _mt_current_goal __mt_current_goal)
   (setq _mt_pwm_goal __mt_pwm_goal)
   self)
  (:mt_operation_mode
   (&optional __mt_operation_mode)
   (if __mt_operation_mode (setq _mt_operation_mode __mt_operation_mode)) _mt_operation_mode)
  (:mt_position_goal
   (&optional __mt_position_goal)
   (if __mt_position_goal (setq _mt_position_goal __mt_position_goal)) _mt_position_goal)
  (:mt_speed_goal
   (&optional __mt_speed_goal)
   (if __mt_speed_goal (setq _mt_speed_goal __mt_speed_goal)) _mt_speed_goal)
  (:mt_current_goal
   (&optional __mt_current_goal)
   (if __mt_current_goal (setq _mt_current_goal __mt_current_goal)) _mt_current_goal)
  (:mt_pwm_goal
   (&optional __mt_pwm_goal)
   (if __mt_pwm_goal (setq _mt_pwm_goal __mt_pwm_goal)) _mt_pwm_goal)
  (:serialization-length
   ()
   (+
    ;; int8[8] _mt_operation_mode
    (* 1    8)
    ;; int32[8] _mt_position_goal
    (* 4    8)
    ;; int32[8] _mt_speed_goal
    (* 4    8)
    ;; int32[8] _mt_current_goal
    (* 4    8)
    ;; int32[8] _mt_pwm_goal
    (* 4    8)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8[8] _mt_operation_mode
     (dotimes (i 8)
       (write-byte (elt _mt_operation_mode i) s)
       )
     ;; int32[8] _mt_position_goal
     (dotimes (i 8)
       (write-long (elt _mt_position_goal i) s)
       )
     ;; int32[8] _mt_speed_goal
     (dotimes (i 8)
       (write-long (elt _mt_speed_goal i) s)
       )
     ;; int32[8] _mt_current_goal
     (dotimes (i 8)
       (write-long (elt _mt_current_goal i) s)
       )
     ;; int32[8] _mt_pwm_goal
     (dotimes (i 8)
       (write-long (elt _mt_pwm_goal i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8[8] _mt_operation_mode
   (dotimes (i (length _mt_operation_mode))
     (setf (elt _mt_operation_mode i) (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> (elt _mt_operation_mode i) 127) (setf (elt _mt_operation_mode i) (- (elt _mt_operation_mode i) 256)))
     )
   ;; int32[8] _mt_position_goal
   (dotimes (i (length _mt_position_goal))
     (setf (elt _mt_position_goal i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;; int32[8] _mt_speed_goal
   (dotimes (i (length _mt_speed_goal))
     (setf (elt _mt_speed_goal i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;; int32[8] _mt_current_goal
   (dotimes (i (length _mt_current_goal))
     (setf (elt _mt_current_goal i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;; int32[8] _mt_pwm_goal
   (dotimes (i (length _mt_pwm_goal))
     (setf (elt _mt_pwm_goal i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;;
   self)
  )

(setf (get bmirobot_msg::motorv2_ctr :md5sum-) "28290c9afcbb7e417ea5e1d8b11ed1b4")
(setf (get bmirobot_msg::motorv2_ctr :datatype-) "bmirobot_msg/motorv2_ctr")
(setf (get bmirobot_msg::motorv2_ctr :definition-)
      "int8[8] mt_operation_mode
int32[8] mt_position_goal
int32[8] mt_speed_goal
int32[8] mt_current_goal
int32[8] mt_pwm_goal

")



(provide :bmirobot_msg/motorv2_ctr "28290c9afcbb7e417ea5e1d8b11ed1b4")


