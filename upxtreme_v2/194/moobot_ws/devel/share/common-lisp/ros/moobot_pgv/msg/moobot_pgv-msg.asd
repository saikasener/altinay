
(cl:in-package :asdf)

(defsystem "moobot_pgv-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "pgv_scan_data" :depends-on ("_package_pgv_scan_data"))
    (:file "_package_pgv_scan_data" :depends-on ("_package"))
  ))