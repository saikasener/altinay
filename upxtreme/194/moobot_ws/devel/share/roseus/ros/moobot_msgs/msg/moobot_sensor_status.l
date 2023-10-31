;; Auto-generated. Do not edit!


(when (boundp 'moobot_msgs::moobot_sensor_status)
  (if (not (find-package "MOOBOT_MSGS"))
    (make-package "MOOBOT_MSGS"))
  (shadow 'moobot_sensor_status (find-package "MOOBOT_MSGS")))
(unless (find-package "MOOBOT_MSGS::MOOBOT_SENSOR_STATUS")
  (make-package "MOOBOT_MSGS::MOOBOT_SENSOR_STATUS"))

(in-package "ROS")
;;//! \htmlinclude moobot_sensor_status.msg.html


(defclass moobot_msgs::moobot_sensor_status
  :super ros::object
  :slots (_state _state_type ))

(defmethod moobot_msgs::moobot_sensor_status
  (:init
   (&key
    ((:state __state) "")
    ((:state_type __state_type) 0)
    )
   (send-super :init)
   (setq _state (string __state))
   (setq _state_type (round __state_type))
   self)
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:state_type
   (&optional __state_type)
   (if __state_type (setq _state_type __state_type)) _state_type)
  (:serialization-length
   ()
   (+
    ;; string _state
    4 (length _state)
    ;; int16 _state_type
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _state
       (write-long (length _state) s) (princ _state s)
     ;; int16 _state_type
       (write-word _state_type s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _state
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _state (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int16 _state_type
     (setq _state_type (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get moobot_msgs::moobot_sensor_status :md5sum-) "49c3408f94118958df8cb878e67805f5")
(setf (get moobot_msgs::moobot_sensor_status :datatype-) "moobot_msgs/moobot_sensor_status")
(setf (get moobot_msgs::moobot_sensor_status :definition-)
      "string state
int16 state_type
")



(provide :moobot_msgs/moobot_sensor_status "49c3408f94118958df8cb878e67805f5")


