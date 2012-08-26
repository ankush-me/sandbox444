
(cl:in-package :asdf)

(defsystem "surgical_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "Cut" :depends-on ("_package_Cut"))
    (:file "_package_Cut" :depends-on ("_package"))
    (:file "Hole" :depends-on ("_package_Hole"))
    (:file "_package_Hole" :depends-on ("_package"))
    (:file "InitInfo" :depends-on ("_package_InitInfo"))
    (:file "_package_InitInfo" :depends-on ("_package"))
  ))