;; Auto-generated. Do not edit!


(when (boundp 'moobot_ui::station)
  (if (not (find-package "MOOBOT_UI"))
    (make-package "MOOBOT_UI"))
  (shadow 'station (find-package "MOOBOT_UI")))
(unless (find-package "MOOBOT_UI::STATION")
  (make-package "MOOBOT_UI::STATION"))

(in-package "ROS")
;;//! \htmlinclude station.msg.html


(defclass moobot_ui::station
  :super ros::object
  :slots (_station_number _x _y _z ))

(defmethod moobot_ui::station
  (:init
   (&key
    ((:station_number __station_number) 0)
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    )
   (send-super :init)
   (setq _station_number (round __station_number))
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   self)
  (:station_number
   (&optional __station_number)
   (if __station_number (setq _station_number __station_number)) _station_number)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:serialization-length
   ()
   (+
    ;; int32 _station_number
    4
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _z
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _station_number
       (write-long _station_number s)
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _z
       (sys::poke _z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _station_number
     (setq _station_number (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _z
     (setq _z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get moobot_ui::station :md5sum-) "5c4f6212c6e666a3f4185ee1f58383a2")
(setf (get moobot_ui::station :datatype-) "moobot_ui/station")
(setf (get moobot_ui::station :definition-)
      "int32 station_number
float32 x
float32 y
float32 z

")



(provide :moobot_ui/station "5c4f6212c6e666a3f4185ee1f58383a2")


