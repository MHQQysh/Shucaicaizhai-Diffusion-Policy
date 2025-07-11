;; Auto-generated. Do not edit!


(when (boundp 'bmirobot_msg::Robot_ctr)
  (if (not (find-package "BMIROBOT_MSG"))
    (make-package "BMIROBOT_MSG"))
  (shadow 'Robot_ctr (find-package "BMIROBOT_MSG")))
(unless (find-package "BMIROBOT_MSG::ROBOT_CTR")
  (make-package "BMIROBOT_MSG::ROBOT_CTR"))

(in-package "ROS")
;;//! \htmlinclude Robot_ctr.msg.html


(defclass bmirobot_msg::Robot_ctr
  :super ros::object
  :slots (_mtID _mtmode _mtpst _mtspd _mttq ))

(defmethod bmirobot_msg::Robot_ctr
  (:init
   (&key
    ((:mtID __mtID) (make-array 9 :initial-element 0 :element-type :integer))
    ((:mtmode __mtmode) (make-array 9 :initial-element 0 :element-type :integer))
    ((:mtpst __mtpst) (make-array 9 :initial-element 0 :element-type :integer))
    ((:mtspd __mtspd) (make-array 9 :initial-element 0 :element-type :integer))
    ((:mttq __mttq) (make-array 9 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _mtID __mtID)
   (setq _mtmode __mtmode)
   (setq _mtpst __mtpst)
   (setq _mtspd __mtspd)
   (setq _mttq __mttq)
   self)
  (:mtID
   (&optional __mtID)
   (if __mtID (setq _mtID __mtID)) _mtID)
  (:mtmode
   (&optional __mtmode)
   (if __mtmode (setq _mtmode __mtmode)) _mtmode)
  (:mtpst
   (&optional __mtpst)
   (if __mtpst (setq _mtpst __mtpst)) _mtpst)
  (:mtspd
   (&optional __mtspd)
   (if __mtspd (setq _mtspd __mtspd)) _mtspd)
  (:mttq
   (&optional __mttq)
   (if __mttq (setq _mttq __mttq)) _mttq)
  (:serialization-length
   ()
   (+
    ;; int32[9] _mtID
    (* 4    9)
    ;; int32[9] _mtmode
    (* 4    9)
    ;; int32[9] _mtpst
    (* 4    9)
    ;; int32[9] _mtspd
    (* 4    9)
    ;; int32[9] _mttq
    (* 4    9)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32[9] _mtID
     (dotimes (i 9)
       (write-long (elt _mtID i) s)
       )
     ;; int32[9] _mtmode
     (dotimes (i 9)
       (write-long (elt _mtmode i) s)
       )
     ;; int32[9] _mtpst
     (dotimes (i 9)
       (write-long (elt _mtpst i) s)
       )
     ;; int32[9] _mtspd
     (dotimes (i 9)
       (write-long (elt _mtspd i) s)
       )
     ;; int32[9] _mttq
     (dotimes (i 9)
       (write-long (elt _mttq i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32[9] _mtID
   (dotimes (i (length _mtID))
     (setf (elt _mtID i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;; int32[9] _mtmode
   (dotimes (i (length _mtmode))
     (setf (elt _mtmode i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;; int32[9] _mtpst
   (dotimes (i (length _mtpst))
     (setf (elt _mtpst i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;; int32[9] _mtspd
   (dotimes (i (length _mtspd))
     (setf (elt _mtspd i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;; int32[9] _mttq
   (dotimes (i (length _mttq))
     (setf (elt _mttq i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;;
   self)
  )

(setf (get bmirobot_msg::Robot_ctr :md5sum-) "0fc2a2db85d9265b43f59ed7bec2ae3c")
(setf (get bmirobot_msg::Robot_ctr :datatype-) "bmirobot_msg/Robot_ctr")
(setf (get bmirobot_msg::Robot_ctr :definition-)
      "int32[9] mtID
int32[9] mtmode
int32[9] mtpst
int32[9] mtspd
int32[9] mttq

")



(provide :bmirobot_msg/Robot_ctr "0fc2a2db85d9265b43f59ed7bec2ae3c")


