;; Auto-generated. Do not edit!


(when (boundp 'control_try::robot)
  (if (not (find-package "CONTROL_TRY"))
    (make-package "CONTROL_TRY"))
  (shadow 'robot (find-package "CONTROL_TRY")))
(unless (find-package "CONTROL_TRY::ROBOT")
  (make-package "CONTROL_TRY::ROBOT"))

(in-package "ROS")
;;//! \htmlinclude robot.msg.html


(defclass control_try::robot
  :super ros::object
  :slots (_joint_pos _joint_spd _eff_pose _eff_spd _timestamp ))

(defmethod control_try::robot
  (:init
   (&key
    ((:joint_pos __joint_pos) (make-array 9 :initial-element 0.0 :element-type :float))
    ((:joint_spd __joint_spd) (make-array 9 :initial-element 0.0 :element-type :float))
    ((:eff_pose __eff_pose) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:eff_spd __eff_spd) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:timestamp __timestamp) 0.0)
    )
   (send-super :init)
   (setq _joint_pos __joint_pos)
   (setq _joint_spd __joint_spd)
   (setq _eff_pose __eff_pose)
   (setq _eff_spd __eff_spd)
   (setq _timestamp (float __timestamp))
   self)
  (:joint_pos
   (&optional __joint_pos)
   (if __joint_pos (setq _joint_pos __joint_pos)) _joint_pos)
  (:joint_spd
   (&optional __joint_spd)
   (if __joint_spd (setq _joint_spd __joint_spd)) _joint_spd)
  (:eff_pose
   (&optional __eff_pose)
   (if __eff_pose (setq _eff_pose __eff_pose)) _eff_pose)
  (:eff_spd
   (&optional __eff_spd)
   (if __eff_spd (setq _eff_spd __eff_spd)) _eff_spd)
  (:timestamp
   (&optional __timestamp)
   (if __timestamp (setq _timestamp __timestamp)) _timestamp)
  (:serialization-length
   ()
   (+
    ;; float64[9] _joint_pos
    (* 8    9)
    ;; float64[9] _joint_spd
    (* 8    9)
    ;; float64[6] _eff_pose
    (* 8    6)
    ;; float64[6] _eff_spd
    (* 8    6)
    ;; float64 _timestamp
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[9] _joint_pos
     (dotimes (i 9)
       (sys::poke (elt _joint_pos i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[9] _joint_spd
     (dotimes (i 9)
       (sys::poke (elt _joint_spd i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _eff_pose
     (dotimes (i 6)
       (sys::poke (elt _eff_pose i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _eff_spd
     (dotimes (i 6)
       (sys::poke (elt _eff_spd i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64 _timestamp
       (sys::poke _timestamp (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[9] _joint_pos
   (dotimes (i (length _joint_pos))
     (setf (elt _joint_pos i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[9] _joint_spd
   (dotimes (i (length _joint_spd))
     (setf (elt _joint_spd i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _eff_pose
   (dotimes (i (length _eff_pose))
     (setf (elt _eff_pose i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _eff_spd
   (dotimes (i (length _eff_spd))
     (setf (elt _eff_spd i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64 _timestamp
     (setq _timestamp (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get control_try::robot :md5sum-) "40052a2da386ad976e3644364a1e9279")
(setf (get control_try::robot :datatype-) "control_try/robot")
(setf (get control_try::robot :definition-)
      "float64[9] joint_pos
float64[9] joint_spd
float64[6] eff_pose
float64[6] eff_spd
float64 timestamp

")



(provide :control_try/robot "40052a2da386ad976e3644364a1e9279")


