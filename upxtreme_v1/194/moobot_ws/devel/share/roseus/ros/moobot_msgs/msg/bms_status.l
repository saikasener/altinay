;; Auto-generated. Do not edit!


(when (boundp 'moobot_msgs::bms_status)
  (if (not (find-package "MOOBOT_MSGS"))
    (make-package "MOOBOT_MSGS"))
  (shadow 'bms_status (find-package "MOOBOT_MSGS")))
(unless (find-package "MOOBOT_MSGS::BMS_STATUS")
  (make-package "MOOBOT_MSGS::BMS_STATUS"))

(in-package "ROS")
;;//! \htmlinclude bms_status.msg.html


(defclass moobot_msgs::bms_status
  :super ros::object
  :slots (_battery_volt_1 _battery_volt_2 _battery_volt_total _battery_temp_1 _battery_temp_2 _current_main _total_power_wh ))

(defmethod moobot_msgs::bms_status
  (:init
   (&key
    ((:battery_volt_1 __battery_volt_1) 0.0)
    ((:battery_volt_2 __battery_volt_2) 0.0)
    ((:battery_volt_total __battery_volt_total) 0.0)
    ((:battery_temp_1 __battery_temp_1) 0.0)
    ((:battery_temp_2 __battery_temp_2) 0.0)
    ((:current_main __current_main) 0.0)
    ((:total_power_wh __total_power_wh) 0.0)
    )
   (send-super :init)
   (setq _battery_volt_1 (float __battery_volt_1))
   (setq _battery_volt_2 (float __battery_volt_2))
   (setq _battery_volt_total (float __battery_volt_total))
   (setq _battery_temp_1 (float __battery_temp_1))
   (setq _battery_temp_2 (float __battery_temp_2))
   (setq _current_main (float __current_main))
   (setq _total_power_wh (float __total_power_wh))
   self)
  (:battery_volt_1
   (&optional __battery_volt_1)
   (if __battery_volt_1 (setq _battery_volt_1 __battery_volt_1)) _battery_volt_1)
  (:battery_volt_2
   (&optional __battery_volt_2)
   (if __battery_volt_2 (setq _battery_volt_2 __battery_volt_2)) _battery_volt_2)
  (:battery_volt_total
   (&optional __battery_volt_total)
   (if __battery_volt_total (setq _battery_volt_total __battery_volt_total)) _battery_volt_total)
  (:battery_temp_1
   (&optional __battery_temp_1)
   (if __battery_temp_1 (setq _battery_temp_1 __battery_temp_1)) _battery_temp_1)
  (:battery_temp_2
   (&optional __battery_temp_2)
   (if __battery_temp_2 (setq _battery_temp_2 __battery_temp_2)) _battery_temp_2)
  (:current_main
   (&optional __current_main)
   (if __current_main (setq _current_main __current_main)) _current_main)
  (:total_power_wh
   (&optional __total_power_wh)
   (if __total_power_wh (setq _total_power_wh __total_power_wh)) _total_power_wh)
  (:serialization-length
   ()
   (+
    ;; float32 _battery_volt_1
    4
    ;; float32 _battery_volt_2
    4
    ;; float32 _battery_volt_total
    4
    ;; float32 _battery_temp_1
    4
    ;; float32 _battery_temp_2
    4
    ;; float32 _current_main
    4
    ;; float32 _total_power_wh
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _battery_volt_1
       (sys::poke _battery_volt_1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _battery_volt_2
       (sys::poke _battery_volt_2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _battery_volt_total
       (sys::poke _battery_volt_total (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _battery_temp_1
       (sys::poke _battery_temp_1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _battery_temp_2
       (sys::poke _battery_temp_2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _current_main
       (sys::poke _current_main (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _total_power_wh
       (sys::poke _total_power_wh (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _battery_volt_1
     (setq _battery_volt_1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _battery_volt_2
     (setq _battery_volt_2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _battery_volt_total
     (setq _battery_volt_total (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _battery_temp_1
     (setq _battery_temp_1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _battery_temp_2
     (setq _battery_temp_2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _current_main
     (setq _current_main (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _total_power_wh
     (setq _total_power_wh (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get moobot_msgs::bms_status :md5sum-) "3113256c57eafd1210064dbad908f61d")
(setf (get moobot_msgs::bms_status :datatype-) "moobot_msgs/bms_status")
(setf (get moobot_msgs::bms_status :definition-)
      "float32 battery_volt_1
float32 battery_volt_2
float32 battery_volt_total
float32 battery_temp_1
float32 battery_temp_2
float32 current_main
float32 total_power_wh

")



(provide :moobot_msgs/bms_status "3113256c57eafd1210064dbad908f61d")


