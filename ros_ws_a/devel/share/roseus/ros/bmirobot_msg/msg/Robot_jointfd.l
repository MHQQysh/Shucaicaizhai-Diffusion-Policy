;; Auto-generated. Do not edit!


(when (boundp 'bmirobot_msg::Robot_jointfd)
  (if (not (find-package "BMIROBOT_MSG"))
    (make-package "BMIROBOT_MSG"))
  (shadow 'Robot_jointfd (find-package "BMIROBOT_MSG")))
(unless (find-package "BMIROBOT_MSG::ROBOT_JOINTFD")
  (make-package "BMIROBOT_MSG::ROBOT_JOINTFD"))

(in-package "ROS")
;;//! \htmlinclude Robot_jointfd.msg.html


(defclass bmirobot_msg::Robot_jointfd
  :super ros::object
  :slots (_Joint_fdpst _Joint_fdspd _Joint_fdctr ))

(defmethod bmirobot_msg::Robot_jointfd
  (:init
   (&key
    ((:Joint_fdpst __Joint_fdpst) (make-array 9 :initial-element 0.0 :element-type :float))
    ((:Joint_fdspd __Joint_fdspd) (make-array 9 :initial-element 0.0 :element-type :float))
    ((:Joint_fdctr __Joint_fdctr) (make-array 9 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _Joint_fdpst __Joint_fdpst)
   (setq _Joint_fdspd __Joint_fdspd)
   (setq _Joint_fdctr __Joint_fdctr)
   self)
  (:Joint_fdpst
   (&optional __Joint_fdpst)
   (if __Joint_fdpst (setq _Joint_fdpst __Joint_fdpst)) _Joint_fdpst)
  (:Joint_fdspd
   (&optional __Joint_fdspd)
   (if __Joint_fdspd (setq _Joint_fdspd __Joint_fdspd)) _Joint_fdspd)
  (:Joint_fdctr
   (&optional __Joint_fdctr)
   (if __Joint_fdctr (setq _Joint_fdctr __Joint_fdctr)) _Joint_fdctr)
  (:serialization-length
   ()
   (+
    ;; float32[9] _Joint_fdpst
    (* 4    9)
    ;; float32[9] _Joint_fdspd
    (* 4    9)
    ;; float32[9] _Joint_fdctr
    (* 4    9)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[9] _Joint_fdpst
     (dotimes (i 9)
       (sys::poke (elt _Joint_fdpst i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[9] _Joint_fdspd
     (dotimes (i 9)
       (sys::poke (elt _Joint_fdspd i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[9] _Joint_fdctr
     (dotimes (i 9)
       (sys::poke (elt _Joint_fdctr i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[9] _Joint_fdpst
   (dotimes (i (length _Joint_fdpst))
     (setf (elt _Joint_fdpst i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[9] _Joint_fdspd
   (dotimes (i (length _Joint_fdspd))
     (setf (elt _Joint_fdspd i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[9] _Joint_fdctr
   (dotimes (i (length _Joint_fdctr))
     (setf (elt _Joint_fdctr i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;;
   self)
  )

(setf (get bmirobot_msg::Robot_jointfd :md5sum-) "8b60b3db716c9968ae69daf16554d81f")
(setf (get bmirobot_msg::Robot_jointfd :datatype-) "bmirobot_msg/Robot_jointfd")
(setf (get bmirobot_msg::Robot_jointfd :definition-)
      "float32[9] Joint_fdpst
float32[9] Joint_fdspd
float32[9] Joint_fdctr


")



(provide :bmirobot_msg/Robot_jointfd "8b60b3db716c9968ae69daf16554d81f")


