
(cl:in-package :asdf)

(defsystem "eval_diffusion-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "robot" :depends-on ("_package_robot"))
    (:file "_package_robot" :depends-on ("_package"))
  ))