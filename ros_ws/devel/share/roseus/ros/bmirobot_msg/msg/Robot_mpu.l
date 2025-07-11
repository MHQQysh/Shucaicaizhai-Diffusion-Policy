;; Auto-generated. Do not edit!


(when (boundp 'bmirobot_msg::Robot_mpu)
  (if (not (find-package "BMIROBOT_MSG"))
    (make-package "BMIROBOT_MSG"))
  (shadow 'Robot_mpu (find-package "BMIROBOT_MSG")))
(unless (find-package "BMIROBOT_MSG::ROBOT_MPU")
  (make-package "BMIROBOT_MSG::ROBOT_MPU"))

(in-package "ROS")
;;//! \htmlinclude Robot_mpu.msg.html


(defclass bmirobot_msg::Robot_mpu
  :super ros::object
  :slots (_mpu_Ax _mpu_Ay _mpu_Az _mpu_Rx _mpu_Ry _mpu_Rz ))

(defmethod bmirobot_msg::Robot_mpu
  (:init
   (&key
    ((:mpu_Ax __mpu_Ax) (make-array 8 :initial-element 0 :element-type :integer))
    ((:mpu_Ay __mpu_Ay) (make-array 8 :initial-element 0 :element-type :integer))
    ((:mpu_Az __mpu_Az) (make-array 8 :initial-element 0 :element-type :integer))
    ((:mpu_Rx __mpu_Rx) (make-array 8 :initial-element 0 :element-type :integer))
    ((:mpu_Ry __mpu_Ry) (make-array 8 :initial-element 0 :element-type :integer))
    ((:mpu_Rz __mpu_Rz) (make-array 8 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _mpu_Ax __mpu_Ax)
   (setq _mpu_Ay __mpu_Ay)
   (setq _mpu_Az __mpu_Az)
   (setq _mpu_Rx __mpu_Rx)
   (setq _mpu_Ry __mpu_Ry)
   (setq _mpu_Rz __mpu_Rz)
   self)
  (:mpu_Ax
   (&optional __mpu_Ax)
   (if __mpu_Ax (setq _mpu_Ax __mpu_Ax)) _mpu_Ax)
  (:mpu_Ay
   (&optional __mpu_Ay)
   (if __mpu_Ay (setq _mpu_Ay __mpu_Ay)) _mpu_Ay)
  (:mpu_Az
   (&optional __mpu_Az)
   (if __mpu_Az (setq _mpu_Az __mpu_Az)) _mpu_Az)
  (:mpu_Rx
   (&optional __mpu_Rx)
   (if __mpu_Rx (setq _mpu_Rx __mpu_Rx)) _mpu_Rx)
  (:mpu_Ry
   (&optional __mpu_Ry)
   (if __mpu_Ry (setq _mpu_Ry __mpu_Ry)) _mpu_Ry)
  (:mpu_Rz
   (&optional __mpu_Rz)
   (if __mpu_Rz (setq _mpu_Rz __mpu_Rz)) _mpu_Rz)
  (:serialization-length
   ()
   (+
    ;; int16[8] _mpu_Ax
    (* 2    8)
    ;; int16[8] _mpu_Ay
    (* 2    8)
    ;; int16[8] _mpu_Az
    (* 2    8)
    ;; int16[8] _mpu_Rx
    (* 2    8)
    ;; int16[8] _mpu_Ry
    (* 2    8)
    ;; int16[8] _mpu_Rz
    (* 2    8)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16[8] _mpu_Ax
     (dotimes (i 8)
       (write-word (elt _mpu_Ax i) s)
       )
     ;; int16[8] _mpu_Ay
     (dotimes (i 8)
       (write-word (elt _mpu_Ay i) s)
       )
     ;; int16[8] _mpu_Az
     (dotimes (i 8)
       (write-word (elt _mpu_Az i) s)
       )
     ;; int16[8] _mpu_Rx
     (dotimes (i 8)
       (write-word (elt _mpu_Rx i) s)
       )
     ;; int16[8] _mpu_Ry
     (dotimes (i 8)
       (write-word (elt _mpu_Ry i) s)
       )
     ;; int16[8] _mpu_Rz
     (dotimes (i 8)
       (write-word (elt _mpu_Rz i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16[8] _mpu_Ax
   (dotimes (i (length _mpu_Ax))
     (setf (elt _mpu_Ax i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;; int16[8] _mpu_Ay
   (dotimes (i (length _mpu_Ay))
     (setf (elt _mpu_Ay i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;; int16[8] _mpu_Az
   (dotimes (i (length _mpu_Az))
     (setf (elt _mpu_Az i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;; int16[8] _mpu_Rx
   (dotimes (i (length _mpu_Rx))
     (setf (elt _mpu_Rx i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;; int16[8] _mpu_Ry
   (dotimes (i (length _mpu_Ry))
     (setf (elt _mpu_Ry i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;; int16[8] _mpu_Rz
   (dotimes (i (length _mpu_Rz))
     (setf (elt _mpu_Rz i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;;
   self)
  )

(setf (get bmirobot_msg::Robot_mpu :md5sum-) "742fca3c0a78013d1f45f4495e1ad202")
(setf (get bmirobot_msg::Robot_mpu :datatype-) "bmirobot_msg/Robot_mpu")
(setf (get bmirobot_msg::Robot_mpu :definition-)
      "int16[8] mpu_Ax
int16[8] mpu_Ay
int16[8] mpu_Az
int16[8] mpu_Rx
int16[8] mpu_Ry
int16[8] mpu_Rz

")



(provide :bmirobot_msg/Robot_mpu "742fca3c0a78013d1f45f4495e1ad202")


