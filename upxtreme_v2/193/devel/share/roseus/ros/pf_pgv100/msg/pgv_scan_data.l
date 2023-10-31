;; Auto-generated. Do not edit!


(when (boundp 'pf_pgv100::pgv_scan_data)
  (if (not (find-package "PF_PGV100"))
    (make-package "PF_PGV100"))
  (shadow 'pgv_scan_data (find-package "PF_PGV100")))
(unless (find-package "PF_PGV100::PGV_SCAN_DATA")
  (make-package "PF_PGV100::PGV_SCAN_DATA"))

(in-package "ROS")
;;//! \htmlinclude pgv_scan_data.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass pf_pgv100::pgv_scan_data
  :super ros::object
  :slots (_header _angle _x_pos _y_pos _direction _color_lane_count _no_color_lane _no_pos _tag_detected _id ))

(defmethod pf_pgv100::pgv_scan_data
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:angle __angle) 0.0)
    ((:x_pos __x_pos) 0.0)
    ((:y_pos __y_pos) 0.0)
    ((:direction __direction) "")
    ((:color_lane_count __color_lane_count) 0)
    ((:no_color_lane __no_color_lane) 0)
    ((:no_pos __no_pos) 0)
    ((:tag_detected __tag_detected) 0)
    ((:id __id) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _angle (float __angle))
   (setq _x_pos (float __x_pos))
   (setq _y_pos (float __y_pos))
   (setq _direction (string __direction))
   (setq _color_lane_count (round __color_lane_count))
   (setq _no_color_lane (round __no_color_lane))
   (setq _no_pos (round __no_pos))
   (setq _tag_detected (round __tag_detected))
   (setq _id (round __id))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:x_pos
   (&optional __x_pos)
   (if __x_pos (setq _x_pos __x_pos)) _x_pos)
  (:y_pos
   (&optional __y_pos)
   (if __y_pos (setq _y_pos __y_pos)) _y_pos)
  (:direction
   (&optional __direction)
   (if __direction (setq _direction __direction)) _direction)
  (:color_lane_count
   (&optional __color_lane_count)
   (if __color_lane_count (setq _color_lane_count __color_lane_count)) _color_lane_count)
  (:no_color_lane
   (&optional __no_color_lane)
   (if __no_color_lane (setq _no_color_lane __no_color_lane)) _no_color_lane)
  (:no_pos
   (&optional __no_pos)
   (if __no_pos (setq _no_pos __no_pos)) _no_pos)
  (:tag_detected
   (&optional __tag_detected)
   (if __tag_detected (setq _tag_detected __tag_detected)) _tag_detected)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _angle
    4
    ;; float32 _x_pos
    4
    ;; float32 _y_pos
    4
    ;; string _direction
    4 (length _direction)
    ;; uint8 _color_lane_count
    1
    ;; uint8 _no_color_lane
    1
    ;; uint8 _no_pos
    1
    ;; uint8 _tag_detected
    1
    ;; uint8 _id
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _angle
       (sys::poke _angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _x_pos
       (sys::poke _x_pos (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y_pos
       (sys::poke _y_pos (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; string _direction
       (write-long (length _direction) s) (princ _direction s)
     ;; uint8 _color_lane_count
       (write-byte _color_lane_count s)
     ;; uint8 _no_color_lane
       (write-byte _no_color_lane s)
     ;; uint8 _no_pos
       (write-byte _no_pos s)
     ;; uint8 _tag_detected
       (write-byte _tag_detected s)
     ;; uint8 _id
       (write-byte _id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _angle
     (setq _angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _x_pos
     (setq _x_pos (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y_pos
     (setq _y_pos (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; string _direction
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _direction (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint8 _color_lane_count
     (setq _color_lane_count (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _no_color_lane
     (setq _no_color_lane (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _no_pos
     (setq _no_pos (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _tag_detected
     (setq _tag_detected (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _id
     (setq _id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get pf_pgv100::pgv_scan_data :md5sum-) "7d3a20e54967aa3cdd07e4fe0a6961b2")
(setf (get pf_pgv100::pgv_scan_data :datatype-) "pf_pgv100/pgv_scan_data")
(setf (get pf_pgv100::pgv_scan_data :definition-)
      "Header header
float32 angle
float32 x_pos
float32 y_pos
string direction
uint8 color_lane_count
uint8 no_color_lane
uint8 no_pos
uint8 tag_detected
uint8 id

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :pf_pgv100/pgv_scan_data "7d3a20e54967aa3cdd07e4fe0a6961b2")


