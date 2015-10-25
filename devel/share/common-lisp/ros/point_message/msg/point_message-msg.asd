
(cl:in-package :asdf)

(defsystem "point_message-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "statsMsg" :depends-on ("_package_statsMsg"))
    (:file "_package_statsMsg" :depends-on ("_package"))
    (:file "pointMsg" :depends-on ("_package_pointMsg"))
    (:file "_package_pointMsg" :depends-on ("_package"))
  ))