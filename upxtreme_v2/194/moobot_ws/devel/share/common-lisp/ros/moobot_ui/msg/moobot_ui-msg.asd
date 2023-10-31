
(cl:in-package :asdf)

(defsystem "moobot_ui-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "agv_status" :depends-on ("_package_agv_status"))
    (:file "_package_agv_status" :depends-on ("_package"))
    (:file "check_imu" :depends-on ("_package_check_imu"))
    (:file "_package_check_imu" :depends-on ("_package"))
    (:file "pid_msg" :depends-on ("_package_pid_msg"))
    (:file "_package_pid_msg" :depends-on ("_package"))
    (:file "station" :depends-on ("_package_station"))
    (:file "_package_station" :depends-on ("_package"))
    (:file "waypoint" :depends-on ("_package_waypoint"))
    (:file "_package_waypoint" :depends-on ("_package"))
  ))