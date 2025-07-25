;; Auto-generated. Do not edit!


(when (boundp 'bmirobot_msg::Robot_fdstatus)
  (if (not (find-package "BMIROBOT_MSG"))
    (make-package "BMIROBOT_MSG"))
  (shadow 'Robot_fdstatus (find-package "BMIROBOT_MSG")))
(unless (find-package "BMIROBOT_MSG::ROBOT_FDSTATUS")
  (make-package "BMIROBOT_MSG::ROBOT_FDSTATUS"))

(in-package "ROS")
;;//! \htmlinclude Robot_fdstatus.msg.html


(defclass bmirobot_msg::Robot_fdstatus
  :super ros::object
  :slots (_mt_mode _mt_Gpst _mt_Cpst _mt_Lpst _mt_Gspd _mt_Cspd _mt_Lspd _mt_Gtq _mt_PWMduty _mt_Gtime _mt_Ctime _mt_Rtime _mt_sysclk _mt_smptime _mt_cputmp _mt_mttmp _mt_invlt _mt_incrt _mt_PWMfrq _mt_ecd _mt_ecdcnt ))

(defmethod bmirobot_msg::Robot_fdstatus
  (:init
   (&key
    ((:mt_mode __mt_mode) (make-array 9 :initial-element 0 :element-type :integer))
    ((:mt_Gpst __mt_Gpst) (make-array 9 :initial-element 0 :element-type :integer))
    ((:mt_Cpst __mt_Cpst) (make-array 9 :initial-element 0 :element-type :integer))
    ((:mt_Lpst __mt_Lpst) (make-array 9 :initial-element 0 :element-type :integer))
    ((:mt_Gspd __mt_Gspd) (make-array 9 :initial-element 0 :element-type :integer))
    ((:mt_Cspd __mt_Cspd) (make-array 9 :initial-element 0 :element-type :integer))
    ((:mt_Lspd __mt_Lspd) (make-array 9 :initial-element 0 :element-type :integer))
    ((:mt_Gtq __mt_Gtq) (make-array 9 :initial-element 0 :element-type :integer))
    ((:mt_PWMduty __mt_PWMduty) (make-array 9 :initial-element 0 :element-type :integer))
    ((:mt_Gtime __mt_Gtime) (make-array 9 :initial-element 0 :element-type :integer))
    ((:mt_Ctime __mt_Ctime) (make-array 9 :initial-element 0 :element-type :integer))
    ((:mt_Rtime __mt_Rtime) (make-array 9 :initial-element 0 :element-type :integer))
    ((:mt_sysclk __mt_sysclk) (make-array 9 :initial-element 0 :element-type :integer))
    ((:mt_smptime __mt_smptime) (make-array 9 :initial-element 0 :element-type :integer))
    ((:mt_cputmp __mt_cputmp) (make-array 9 :initial-element 0 :element-type :integer))
    ((:mt_mttmp __mt_mttmp) (make-array 9 :initial-element 0 :element-type :integer))
    ((:mt_invlt __mt_invlt) (make-array 9 :initial-element 0 :element-type :integer))
    ((:mt_incrt __mt_incrt) (make-array 9 :initial-element 0 :element-type :integer))
    ((:mt_PWMfrq __mt_PWMfrq) (make-array 9 :initial-element 0 :element-type :integer))
    ((:mt_ecd __mt_ecd) (make-array 9 :initial-element 0 :element-type :integer))
    ((:mt_ecdcnt __mt_ecdcnt) (make-array 9 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _mt_mode __mt_mode)
   (setq _mt_Gpst __mt_Gpst)
   (setq _mt_Cpst __mt_Cpst)
   (setq _mt_Lpst __mt_Lpst)
   (setq _mt_Gspd __mt_Gspd)
   (setq _mt_Cspd __mt_Cspd)
   (setq _mt_Lspd __mt_Lspd)
   (setq _mt_Gtq __mt_Gtq)
   (setq _mt_PWMduty __mt_PWMduty)
   (setq _mt_Gtime __mt_Gtime)
   (setq _mt_Ctime __mt_Ctime)
   (setq _mt_Rtime __mt_Rtime)
   (setq _mt_sysclk __mt_sysclk)
   (setq _mt_smptime __mt_smptime)
   (setq _mt_cputmp __mt_cputmp)
   (setq _mt_mttmp __mt_mttmp)
   (setq _mt_invlt __mt_invlt)
   (setq _mt_incrt __mt_incrt)
   (setq _mt_PWMfrq __mt_PWMfrq)
   (setq _mt_ecd __mt_ecd)
   (setq _mt_ecdcnt __mt_ecdcnt)
   self)
  (:mt_mode
   (&optional __mt_mode)
   (if __mt_mode (setq _mt_mode __mt_mode)) _mt_mode)
  (:mt_Gpst
   (&optional __mt_Gpst)
   (if __mt_Gpst (setq _mt_Gpst __mt_Gpst)) _mt_Gpst)
  (:mt_Cpst
   (&optional __mt_Cpst)
   (if __mt_Cpst (setq _mt_Cpst __mt_Cpst)) _mt_Cpst)
  (:mt_Lpst
   (&optional __mt_Lpst)
   (if __mt_Lpst (setq _mt_Lpst __mt_Lpst)) _mt_Lpst)
  (:mt_Gspd
   (&optional __mt_Gspd)
   (if __mt_Gspd (setq _mt_Gspd __mt_Gspd)) _mt_Gspd)
  (:mt_Cspd
   (&optional __mt_Cspd)
   (if __mt_Cspd (setq _mt_Cspd __mt_Cspd)) _mt_Cspd)
  (:mt_Lspd
   (&optional __mt_Lspd)
   (if __mt_Lspd (setq _mt_Lspd __mt_Lspd)) _mt_Lspd)
  (:mt_Gtq
   (&optional __mt_Gtq)
   (if __mt_Gtq (setq _mt_Gtq __mt_Gtq)) _mt_Gtq)
  (:mt_PWMduty
   (&optional __mt_PWMduty)
   (if __mt_PWMduty (setq _mt_PWMduty __mt_PWMduty)) _mt_PWMduty)
  (:mt_Gtime
   (&optional __mt_Gtime)
   (if __mt_Gtime (setq _mt_Gtime __mt_Gtime)) _mt_Gtime)
  (:mt_Ctime
   (&optional __mt_Ctime)
   (if __mt_Ctime (setq _mt_Ctime __mt_Ctime)) _mt_Ctime)
  (:mt_Rtime
   (&optional __mt_Rtime)
   (if __mt_Rtime (setq _mt_Rtime __mt_Rtime)) _mt_Rtime)
  (:mt_sysclk
   (&optional __mt_sysclk)
   (if __mt_sysclk (setq _mt_sysclk __mt_sysclk)) _mt_sysclk)
  (:mt_smptime
   (&optional __mt_smptime)
   (if __mt_smptime (setq _mt_smptime __mt_smptime)) _mt_smptime)
  (:mt_cputmp
   (&optional __mt_cputmp)
   (if __mt_cputmp (setq _mt_cputmp __mt_cputmp)) _mt_cputmp)
  (:mt_mttmp
   (&optional __mt_mttmp)
   (if __mt_mttmp (setq _mt_mttmp __mt_mttmp)) _mt_mttmp)
  (:mt_invlt
   (&optional __mt_invlt)
   (if __mt_invlt (setq _mt_invlt __mt_invlt)) _mt_invlt)
  (:mt_incrt
   (&optional __mt_incrt)
   (if __mt_incrt (setq _mt_incrt __mt_incrt)) _mt_incrt)
  (:mt_PWMfrq
   (&optional __mt_PWMfrq)
   (if __mt_PWMfrq (setq _mt_PWMfrq __mt_PWMfrq)) _mt_PWMfrq)
  (:mt_ecd
   (&optional __mt_ecd)
   (if __mt_ecd (setq _mt_ecd __mt_ecd)) _mt_ecd)
  (:mt_ecdcnt
   (&optional __mt_ecdcnt)
   (if __mt_ecdcnt (setq _mt_ecdcnt __mt_ecdcnt)) _mt_ecdcnt)
  (:serialization-length
   ()
   (+
    ;; int32[9] _mt_mode
    (* 4    9)
    ;; int32[9] _mt_Gpst
    (* 4    9)
    ;; int32[9] _mt_Cpst
    (* 4    9)
    ;; int32[9] _mt_Lpst
    (* 4    9)
    ;; int32[9] _mt_Gspd
    (* 4    9)
    ;; int32[9] _mt_Cspd
    (* 4    9)
    ;; int32[9] _mt_Lspd
    (* 4    9)
    ;; int32[9] _mt_Gtq
    (* 4    9)
    ;; int32[9] _mt_PWMduty
    (* 4    9)
    ;; int32[9] _mt_Gtime
    (* 4    9)
    ;; int32[9] _mt_Ctime
    (* 4    9)
    ;; int32[9] _mt_Rtime
    (* 4    9)
    ;; int32[9] _mt_sysclk
    (* 4    9)
    ;; int32[9] _mt_smptime
    (* 4    9)
    ;; int32[9] _mt_cputmp
    (* 4    9)
    ;; int32[9] _mt_mttmp
    (* 4    9)
    ;; int32[9] _mt_invlt
    (* 4    9)
    ;; int32[9] _mt_incrt
    (* 4    9)
    ;; int32[9] _mt_PWMfrq
    (* 4    9)
    ;; int32[9] _mt_ecd
    (* 4    9)
    ;; int32[9] _mt_ecdcnt
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
     ;; int32[9] _mt_Gpst
     (dotimes (i 9)
       (write-long (elt _mt_Gpst i) s)
       )
     ;; int32[9] _mt_Cpst
     (dotimes (i 9)
       (write-long (elt _mt_Cpst i) s)
       )
     ;; int32[9] _mt_Lpst
     (dotimes (i 9)
       (write-long (elt _mt_Lpst i) s)
       )
     ;; int32[9] _mt_Gspd
     (dotimes (i 9)
       (write-long (elt _mt_Gspd i) s)
       )
     ;; int32[9] _mt_Cspd
     (dotimes (i 9)
       (write-long (elt _mt_Cspd i) s)
       )
     ;; int32[9] _mt_Lspd
     (dotimes (i 9)
       (write-long (elt _mt_Lspd i) s)
       )
     ;; int32[9] _mt_Gtq
     (dotimes (i 9)
       (write-long (elt _mt_Gtq i) s)
       )
     ;; int32[9] _mt_PWMduty
     (dotimes (i 9)
       (write-long (elt _mt_PWMduty i) s)
       )
     ;; int32[9] _mt_Gtime
     (dotimes (i 9)
       (write-long (elt _mt_Gtime i) s)
       )
     ;; int32[9] _mt_Ctime
     (dotimes (i 9)
       (write-long (elt _mt_Ctime i) s)
       )
     ;; int32[9] _mt_Rtime
     (dotimes (i 9)
       (write-long (elt _mt_Rtime i) s)
       )
     ;; int32[9] _mt_sysclk
     (dotimes (i 9)
       (write-long (elt _mt_sysclk i) s)
       )
     ;; int32[9] _mt_smptime
     (dotimes (i 9)
       (write-long (elt _mt_smptime i) s)
       )
     ;; int32[9] _mt_cputmp
     (dotimes (i 9)
       (write-long (elt _mt_cputmp i) s)
       )
     ;; int32[9] _mt_mttmp
     (dotimes (i 9)
       (write-long (elt _mt_mttmp i) s)
       )
     ;; int32[9] _mt_invlt
     (dotimes (i 9)
       (write-long (elt _mt_invlt i) s)
       )
     ;; int32[9] _mt_incrt
     (dotimes (i 9)
       (write-long (elt _mt_incrt i) s)
       )
     ;; int32[9] _mt_PWMfrq
     (dotimes (i 9)
       (write-long (elt _mt_PWMfrq i) s)
       )
     ;; int32[9] _mt_ecd
     (dotimes (i 9)
       (write-long (elt _mt_ecd i) s)
       )
     ;; int32[9] _mt_ecdcnt
     (dotimes (i 9)
       (write-long (elt _mt_ecdcnt i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32[9] _mt_mode
   (dotimes (i (length _mt_mode))
     (setf (elt _mt_mode i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;; int32[9] _mt_Gpst
   (dotimes (i (length _mt_Gpst))
     (setf (elt _mt_Gpst i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;; int32[9] _mt_Cpst
   (dotimes (i (length _mt_Cpst))
     (setf (elt _mt_Cpst i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;; int32[9] _mt_Lpst
   (dotimes (i (length _mt_Lpst))
     (setf (elt _mt_Lpst i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;; int32[9] _mt_Gspd
   (dotimes (i (length _mt_Gspd))
     (setf (elt _mt_Gspd i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;; int32[9] _mt_Cspd
   (dotimes (i (length _mt_Cspd))
     (setf (elt _mt_Cspd i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;; int32[9] _mt_Lspd
   (dotimes (i (length _mt_Lspd))
     (setf (elt _mt_Lspd i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;; int32[9] _mt_Gtq
   (dotimes (i (length _mt_Gtq))
     (setf (elt _mt_Gtq i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;; int32[9] _mt_PWMduty
   (dotimes (i (length _mt_PWMduty))
     (setf (elt _mt_PWMduty i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;; int32[9] _mt_Gtime
   (dotimes (i (length _mt_Gtime))
     (setf (elt _mt_Gtime i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;; int32[9] _mt_Ctime
   (dotimes (i (length _mt_Ctime))
     (setf (elt _mt_Ctime i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;; int32[9] _mt_Rtime
   (dotimes (i (length _mt_Rtime))
     (setf (elt _mt_Rtime i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;; int32[9] _mt_sysclk
   (dotimes (i (length _mt_sysclk))
     (setf (elt _mt_sysclk i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;; int32[9] _mt_smptime
   (dotimes (i (length _mt_smptime))
     (setf (elt _mt_smptime i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;; int32[9] _mt_cputmp
   (dotimes (i (length _mt_cputmp))
     (setf (elt _mt_cputmp i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;; int32[9] _mt_mttmp
   (dotimes (i (length _mt_mttmp))
     (setf (elt _mt_mttmp i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;; int32[9] _mt_invlt
   (dotimes (i (length _mt_invlt))
     (setf (elt _mt_invlt i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;; int32[9] _mt_incrt
   (dotimes (i (length _mt_incrt))
     (setf (elt _mt_incrt i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;; int32[9] _mt_PWMfrq
   (dotimes (i (length _mt_PWMfrq))
     (setf (elt _mt_PWMfrq i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;; int32[9] _mt_ecd
   (dotimes (i (length _mt_ecd))
     (setf (elt _mt_ecd i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;; int32[9] _mt_ecdcnt
   (dotimes (i (length _mt_ecdcnt))
     (setf (elt _mt_ecdcnt i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;;
   self)
  )

(setf (get bmirobot_msg::Robot_fdstatus :md5sum-) "96575b502188e773da8e23f3a4f7ae68")
(setf (get bmirobot_msg::Robot_fdstatus :datatype-) "bmirobot_msg/Robot_fdstatus")
(setf (get bmirobot_msg::Robot_fdstatus :definition-)
      "int32[9] mt_mode
int32[9] mt_Gpst
int32[9] mt_Cpst
int32[9] mt_Lpst
int32[9] mt_Gspd
int32[9] mt_Cspd
int32[9] mt_Lspd
int32[9] mt_Gtq
int32[9] mt_PWMduty
int32[9] mt_Gtime
int32[9] mt_Ctime
int32[9] mt_Rtime
int32[9] mt_sysclk
int32[9] mt_smptime
int32[9] mt_cputmp
int32[9] mt_mttmp
int32[9] mt_invlt
int32[9] mt_incrt
int32[9] mt_PWMfrq
int32[9] mt_ecd
int32[9] mt_ecdcnt
")



(provide :bmirobot_msg/Robot_fdstatus "96575b502188e773da8e23f3a4f7ae68")


