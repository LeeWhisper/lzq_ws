
(cl:in-package :asdf)

(defsystem "track_avoid-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Depth" :depends-on ("_package_Depth"))
    (:file "_package_Depth" :depends-on ("_package"))
  ))