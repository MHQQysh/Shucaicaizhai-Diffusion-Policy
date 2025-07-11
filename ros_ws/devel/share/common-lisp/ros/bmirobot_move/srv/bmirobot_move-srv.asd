
(cl:in-package :asdf)

(defsystem "bmirobot_move-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "GraspServo" :depends-on ("_package_GraspServo"))
    (:file "_package_GraspServo" :depends-on ("_package"))
  ))