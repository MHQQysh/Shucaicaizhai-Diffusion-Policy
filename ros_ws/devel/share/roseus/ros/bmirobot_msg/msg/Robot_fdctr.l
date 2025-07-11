;; Auto-generated. Do not edit!


(when (boundp 'bmirobot_msg::Robot_fdctr)
  (if (not (find-package "BMIROBOT_MSG"))
    (make-package "BMIROBOT_MSG"))
  (shadow 'Robot_fdctr (find-package "BMIROBOT_MSG")))
(unless (find-package "BMIROBOT_MSG::ROBOT_FDCTR")
  (make-package "BMIROBOT_MSG::ROBOT_FDCTR"))

(in-package "ROS")
;;//! \htmlinclude Robot_fdctr.msg.html


(defclass bmirobot_msg::Robot_fdctr
  :super ros::object
  :slots (_mt_mode _mt_Cpst _mt_Cspd _mt_incrt _mt_PWMduty ))

(defmethod bmirobot_msg::Robot_fdctr
  (:init
   (&key
    ((:mt_mode __mt_mode) (make-array 9 :initial-element 0 :element-type :integer))
    ((:mt_Cpst __mt_Cpst) (make-array 9 :initial-element 0 :element-type :integer))
    ((:mt_Cspd __mt_Cspd) (make-array 9 :initial-element 0 :element-type :integer))
    ((:mt_incrt __mt_incrt) (make-array 9 :initial-element 0 :element-type :integer))
    ((:mt_PWMduty __mt_PWMduty) (make-array 9 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _mt_mode __mt_mode)
   (setq _mt_Cpst __mt_Cpst)
   (setq _mt_Cspd __mt_Cspd)
   (setq _mt_incrt __mt_incrt)
   (setq _mt_PWMduty __mt_PWMduty)
   self)
  (:mt_mode
   (&optional __mt_mode)
   (if __mt_mode (setq _mt_mode __mt_mode)) _mt_mode)
  (:mt_Cpst
   (&optional __mt_Cpst)
   (if __mt_Cpst (setq _mt_Cpst __mt_Cpst)) _mt_Cpst)
  (:mt_Cspd
   (&optional __mt_Cspd)
   (if __mt_Cspd (setq _mt_Cspd __mt_Cspd)) _mt_Cspd)
  (:mt_incrt
   (&optional __mt_incrt)
   (if __mt_incrt (setq _mt_incrt __mt_incrt)) _mt_incrt)
  (:mt_PWMduty
   (&optional __mt_PWMduty)
   (if __mt_PWMduty (setq _mt_PWMduty __mt_PWMduty)) _mt_PWMduty)
  (:serialization-length
   ()
   (+
    ;; int32[9] _mt_mode
    (* 4    9)
    ;; int32[9] _mt_Cpst
    (* 4    9)
    ;; int32[9] _mt_Cspd
    (* 4    9)
    ;; int32[9] _mt_incrt
    (* 4    9)
    ;; int32[9] _mt_PWMduty
    (* 4    9)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32[9] _mt_mode
     (dotimes (i 9)
       (write-long (elt _mt_mode i) s)
       )
     ;; int32[9] _mt_Cpst
     (dotimes (i 9)
       (write-long (elt _mt_Cpst i) s)
       )
     ;; int32[9] _mt_Cspd
     (dotimes (i 9)
       (write-long (elt _mt_Cspd i) s)
       )
     ;; int32[9] _mt_incrt
     (dotimes (i 9)
       (write-long (elt _mt_incrt i) s)
       )
     ;; int32[9] _mt_PWMduty
     (dotimes (i 9)
       (write-long (elt _mt_PWMduty i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32[9] _mt_mode
   (dotimes (i (length _mt_mode))
     (setf (elt _mt_mode i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;; int32[9] _mt_Cpst
   (dotimes (i (length _mt_Cpst))
     (setf (elt _mt_Cpst i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;; int32[9] _mt_Cspd
   (dotimes (i (length _mt_Cspd))
     (setf (elt _mt_Cspd i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;; int32[9] _mt_incrt
   (dotimes (i (length _mt_incrt))
     (setf (elt _mt_incrt i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;; int32[9] _mt_PWMduty
   (dotimes (i (length _mt_PWMduty))
     (setf (elt _mt_PWMduty i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;;
   self)
  )

(setf (get bmirobot_msg::Robot_fdctr :md5sum-) "3195cf007256e8ddfd66fc62f56f5233")
(setf (get bmirobot_msg::Robot_fdctr :datatype-) "bmirobot_msg/Robot_fdctr")
(setf (get bmirobot_msg::Robot_fdctr :definition-)
      "int32[9] mt_mode
int32[9] mt_Cpst
int32[9] mt_Cspd
int32[9] mt_incrt
int32[9] mt_PWMduty

")



(provide :bmirobot_msg/Robot_fdctr "3195cf007256e8ddfd66fc62f56f5233")


