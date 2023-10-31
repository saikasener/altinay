;; Auto-generated. Do not edit!


(when (boundp 'moobot_msgs::lift_status)
  (if (not (find-package "MOOBOT_MSGS"))
    (make-package "MOOBOT_MSGS"))
  (shadow 'lift_status (find-package "MOOBOT_MSGS")))
(unless (find-package "MOOBOT_MSGS::LIFT_STATUS")
  (make-package "MOOBOT_MSGS::LIFT_STATUS"))

(in-package "ROS")
;;//! \htmlinclude lift_status.msg.html


(defclass moobot_msgs::lift_status
  :super ros::object
  :slots (_up_lift _down_lift ))

(defmethod moobot_msgs::lift_status
  (:init
   (&key
    ((:up_lift __up_lift) nil)
    ((:down_lift __down_lift) nil)
    )
   (send-super :init)
   (setq _up_lift __up_lift)
   (setq _down_lift __down_lift)
   self)
  (:up_lift
   (&optional (__up_lift :null))
   (if (not (eq __up_lift :null)) (setq _up_lift __up_lift)) _up_lift)
  (:down_lift
   (&optional (__down_lift :null))
   (if (not (eq __down_lift :null)) (setq _down_lift __down_lift)) _down_lift)
  (:serialization-length
   ()
   (+
    ;; bool _up_lift
    1
    ;; bool _down_lift
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _up_lift
       (if _up_lift (write-byte -1 s) (write-byte 0 s))
     ;; bool _down_lift
       (if _down_lift (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _up_lift
     (setq _up_lift (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _down_lift
     (setq _down_lift (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get moobot_msgs::lift_status :md5sum-) "88d1b4abfd45857169d18e9ca11502ca")
(setf (get moobot_msgs::lift_status :datatype-) "moobot_msgs/lift_status")
(setf (get moobot_msgs::lift_status :definition-)
      "bool up_lift
bool down_lift

")



(provide :moobot_msgs/lift_status "88d1b4abfd45857169d18e9ca11502ca")


