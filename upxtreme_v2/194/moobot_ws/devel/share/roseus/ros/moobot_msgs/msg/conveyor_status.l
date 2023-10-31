;; Auto-generated. Do not edit!


(when (boundp 'moobot_msgs::conveyor_status)
  (if (not (find-package "MOOBOT_MSGS"))
    (make-package "MOOBOT_MSGS"))
  (shadow 'conveyor_status (find-package "MOOBOT_MSGS")))
(unless (find-package "MOOBOT_MSGS::CONVEYOR_STATUS")
  (make-package "MOOBOT_MSGS::CONVEYOR_STATUS"))

(in-package "ROS")
;;//! \htmlinclude conveyor_status.msg.html


(defclass moobot_msgs::conveyor_status
  :super ros::object
  :slots (_load_conveyor _unload_conveyor ))

(defmethod moobot_msgs::conveyor_status
  (:init
   (&key
    ((:load_conveyor __load_conveyor) nil)
    ((:unload_conveyor __unload_conveyor) nil)
    )
   (send-super :init)
   (setq _load_conveyor __load_conveyor)
   (setq _unload_conveyor __unload_conveyor)
   self)
  (:load_conveyor
   (&optional (__load_conveyor :null))
   (if (not (eq __load_conveyor :null)) (setq _load_conveyor __load_conveyor)) _load_conveyor)
  (:unload_conveyor
   (&optional (__unload_conveyor :null))
   (if (not (eq __unload_conveyor :null)) (setq _unload_conveyor __unload_conveyor)) _unload_conveyor)
  (:serialization-length
   ()
   (+
    ;; bool _load_conveyor
    1
    ;; bool _unload_conveyor
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _load_conveyor
       (if _load_conveyor (write-byte -1 s) (write-byte 0 s))
     ;; bool _unload_conveyor
       (if _unload_conveyor (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _load_conveyor
     (setq _load_conveyor (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _unload_conveyor
     (setq _unload_conveyor (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get moobot_msgs::conveyor_status :md5sum-) "a8462790025e5d19ec5df8dbb71cb071")
(setf (get moobot_msgs::conveyor_status :datatype-) "moobot_msgs/conveyor_status")
(setf (get moobot_msgs::conveyor_status :definition-)
      "bool load_conveyor
bool unload_conveyor
")



(provide :moobot_msgs/conveyor_status "a8462790025e5d19ec5df8dbb71cb071")


