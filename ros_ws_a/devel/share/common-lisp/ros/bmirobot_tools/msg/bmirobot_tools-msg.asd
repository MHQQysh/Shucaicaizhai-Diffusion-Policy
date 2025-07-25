
(cl:in-package :asdf)

(defsystem "bmirobot_tools-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "PointsPR" :depends-on ("_package_PointsPR"))
    (:file "_package_PointsPR" :depends-on ("_package"))
  ))