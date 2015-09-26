
(cl:in-package :asdf)

(defsystem "point_message-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "pointMsg" :depends-on ("_package_pointMsg"))
    (:file "_package_pointMsg" :depends-on ("_package"))
  ))