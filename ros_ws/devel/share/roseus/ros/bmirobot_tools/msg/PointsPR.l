;; Auto-generated. Do not edit!


(when (boundp 'bmirobot_tools::PointsPR)
  (if (not (find-package "BMIROBOT_TOOLS"))
    (make-package "BMIROBOT_TOOLS"))
  (shadow 'PointsPR (find-package "BMIROBOT_TOOLS")))
(unless (find-package "BMIROBOT_TOOLS::POINTSPR")
  (make-package "BMIROBOT_TOOLS::POINTSPR"))

(in-package "ROS")
;;//! \htmlinclude PointsPR.msg.html


(defclass bmirobot_tools::PointsPR
  :super ros::object
  :slots (_Px _Py _Pz _Rx _Ry _Rz ))

(defmethod bmirobot_tools::PointsPR
  (:init
   (&key
    ((:Px __Px) (make-array 4 :initial-element 0.0 :element-type :float))
    ((:Py __Py) (make-array 4 :initial-element 0.0 :element-type :float))
    ((:Pz __Pz) (make-array 4 :initial-element 0.0 :element-type :float))
    ((:Rx __Rx) (make-array 4 :initial-element 0.0 :element-type :float))
    ((:Ry __Ry) (make-array 4 :initial-element 0.0 :element-type :float))
    ((:Rz __Rz) (make-array 4 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _Px __Px)
   (setq _Py __Py)
   (setq _Pz __Pz)
   (setq _Rx __Rx)
   (setq _Ry __Ry)
   (setq _Rz __Rz)
   self)
  (:Px
   (&optional __Px)
   (if __Px (setq _Px __Px)) _Px)
  (:Py
   (&optional __Py)
   (if __Py (setq _Py __Py)) _Py)
  (:Pz
   (&optional __Pz)
   (if __Pz (setq _Pz __Pz)) _Pz)
  (:Rx
   (&optional __Rx)
   (if __Rx (setq _Rx __Rx)) _Rx)
  (:Ry
   (&optional __Ry)
   (if __Ry (setq _Ry __Ry)) _Ry)
  (:Rz
   (&optional __Rz)
   (if __Rz (setq _Rz __Rz)) _Rz)
  (:serialization-length
   ()
   (+
    ;; float64[4] _Px
    (* 8    4)
    ;; float64[4] _Py
    (* 8    4)
    ;; float64[4] _Pz
    (* 8    4)
    ;; float64[4] _Rx
    (* 8    4)
    ;; float64[4] _Ry
    (* 8    4)
    ;; float64[4] _Rz
    (* 8    4)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[4] _Px
     (dotimes (i 4)
       (sys::poke (elt _Px i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[4] _Py
     (dotimes (i 4)
       (sys::poke (elt _Py i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[4] _Pz
     (dotimes (i 4)
       (sys::poke (elt _Pz i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[4] _Rx
     (dotimes (i 4)
       (sys::poke (elt _Rx i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[4] _Ry
     (dotimes (i 4)
       (sys::poke (elt _Ry i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[4] _Rz
     (dotimes (i 4)
       (sys::poke (elt _Rz i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[4] _Px
   (dotimes (i (length _Px))
     (setf (elt _Px i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[4] _Py
   (dotimes (i (length _Py))
     (setf (elt _Py i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[4] _Pz
   (dotimes (i (length _Pz))
     (setf (elt _Pz i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[4] _Rx
   (dotimes (i (length _Rx))
     (setf (elt _Rx i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[4] _Ry
   (dotimes (i (length _Ry))
     (setf (elt _Ry i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[4] _Rz
   (dotimes (i (length _Rz))
     (setf (elt _Rz i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(setf (get bmirobot_tools::PointsPR :md5sum-) "5f5a7d3cfad1346be5c8446784c69a6c")
(setf (get bmirobot_tools::PointsPR :datatype-) "bmirobot_tools/PointsPR")
(setf (get bmirobot_tools::PointsPR :definition-)
      "float64[4] Px
float64[4] Py
float64[4] Pz
float64[4] Rx
float64[4] Ry
float64[4] Rz

")



(provide :bmirobot_tools/PointsPR "5f5a7d3cfad1346be5c8446784c69a6c")


