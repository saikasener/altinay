;; Auto-generated. Do not edit!


(when (boundp 'moobot_ui::waypoint)
  (if (not (find-package "MOOBOT_UI"))
    (make-package "MOOBOT_UI"))
  (shadow 'waypoint (find-package "MOOBOT_UI")))
(unless (find-package "MOOBOT_UI::WAYPOINT")
  (make-package "MOOBOT_UI::WAYPOINT"))

(in-package "ROS")
;;//! \htmlinclude waypoint.msg.html


(defclass moobot_ui::waypoint
  :super ros::object
  :slots (_way_point_name _x _y _z ))

(defmethod moobot_ui::waypoint
  (:init
   (&key
    ((:way_point_name __way_point_name) "")
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    )
   (send-super :init)
   (setq _way_point_name (string __way_point_name))
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   self)
  (:way_point_name
   (&optional __way_point_name)
   (if __way_point_name (setq _way_point_name __way_point_name)) _way_point_name)
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
    ;; string _way_point_name
    4 (length _way_point_name)
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
     ;; string _way_point_name
       (write-long (length _way_point_name) s) (princ _way_point_name s)
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
   ;; string _way_point_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _way_point_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _z
     (setq _z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get moobot_ui::waypoint :md5sum-) "a43892a9a5e3559df1641a6a9d36bbe8")
(setf (get moobot_ui::waypoint :datatype-) "moobot_ui/waypoint")
(setf (get moobot_ui::waypoint :definition-)
      "string way_point_name
float32 x
float32 y
float32 z

")



(provide :moobot_ui/waypoint "a43892a9a5e3559df1641a6a9d36bbe8")


