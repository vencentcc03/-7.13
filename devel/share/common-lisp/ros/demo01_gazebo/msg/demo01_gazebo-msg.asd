
(cl:in-package :asdf)

(defsystem "demo01_gazebo-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "coordinate" :depends-on ("_package_coordinate"))
    (:file "_package_coordinate" :depends-on ("_package"))
  ))