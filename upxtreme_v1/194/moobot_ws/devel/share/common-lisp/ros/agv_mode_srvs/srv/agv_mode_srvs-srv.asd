
(cl:in-package :asdf)

(defsystem "agv_mode_srvs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "agv_mode_srv" :depends-on ("_package_agv_mode_srv"))
    (:file "_package_agv_mode_srv" :depends-on ("_package"))
  ))