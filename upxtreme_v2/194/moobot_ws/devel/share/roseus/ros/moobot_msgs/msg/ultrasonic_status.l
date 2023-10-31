;; Auto-generated. Do not edit!


(when (boundp 'moobot_msgs::ultrasonic_status)
  (if (not (find-package "MOOBOT_MSGS"))
    (make-package "MOOBOT_MSGS"))
  (shadow 'ultrasonic_status (find-package "MOOBOT_MSGS")))
(unless (find-package "MOOBOT_MSGS::ULTRASONIC_STATUS")
  (make-package "MOOBOT_MSGS::ULTRASONIC_STATUS"))

(in-package "ROS")
;;//! \htmlinclude ultrasonic_status.msg.html


(defclass moobot_msgs::ultrasonic_status
  :super ros::object
  :slots (_S_Protection ))

(defmethod moobot_msgs::ultrasonic_status
  (:init
   (&key
    ((:S_Protection __S_Protection) nil)
    )
   (send-super :init)
   (setq _S_Protection __S_Protection)
   self)
  (:S_Protection
   (&optional (__S_Protection :null))
   (if (not (eq __S_Protection :null)) (setq _S_Protection __S_Protection)) _S_Protection)
  (:serialization-length
   ()
   (+
    ;; bool _S_Protection
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _S_Protection
       (if _S_Protection (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _S_Protection
     (setq _S_Protection (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get moobot_msgs::ultrasonic_status :md5sum-) "efb803211b7dc44878ac181a026e608e")
(setf (get moobot_msgs::ultrasonic_status :datatype-) "moobot_msgs/ultrasonic_status")
(setf (get moobot_msgs::ultrasonic_status :definition-)
      "bool S_Protection

")



(provide :moobot_msgs/ultrasonic_status "efb803211b7dc44878ac181a026e608e")


