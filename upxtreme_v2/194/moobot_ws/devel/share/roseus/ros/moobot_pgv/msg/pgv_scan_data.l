;; Auto-generated. Do not edit!


(when (boundp 'moobot_pgv::pgv_scan_data)
  (if (not (find-package "MOOBOT_PGV"))
    (make-package "MOOBOT_PGV"))
  (shadow 'pgv_scan_data (find-package "MOOBOT_PGV")))
(unless (find-package "MOOBOT_PGV::PGV_SCAN_DATA")
  (make-package "MOOBOT_PGV::PGV_SCAN_DATA"))

(in-package "ROS")
;;//! \htmlinclude pgv_scan_data.msg.html


(defclass moobot_pgv::pgv_scan_data
  :super ros::object
  :slots (_x_pos _y_pos _orientation _read_barcode _tag_num _lane_detected ))

(defmethod moobot_pgv::pgv_scan_data
  (:init
   (&key
    ((:x_pos __x_pos) 0.0)
    ((:y_pos __y_pos) 0.0)
    ((:orientation __orientation) 0.0)
    ((:read_barcode __read_barcode) nil)
    ((:tag_num __tag_num) 0)
    ((:lane_detected __lane_detected) 0)
    )
   (send-super :init)
   (setq _x_pos (float __x_pos))
   (setq _y_pos (float __y_pos))
   (setq _orientation (float __orientation))
   (setq _read_barcode __read_barcode)
   (setq _tag_num (round __tag_num))
   (setq _lane_detected (round __lane_detected))
   self)
  (:x_pos
   (&optional __x_pos)
   (if __x_pos (setq _x_pos __x_pos)) _x_pos)
  (:y_pos
   (&optional __y_pos)
   (if __y_pos (setq _y_pos __y_pos)) _y_pos)
  (:orientation
   (&optional __orientation)
   (if __orientation (setq _orientation __orientation)) _orientation)
  (:read_barcode
   (&optional (__read_barcode :null))
   (if (not (eq __read_barcode :null)) (setq _read_barcode __read_barcode)) _read_barcode)
  (:tag_num
   (&optional __tag_num)
   (if __tag_num (setq _tag_num __tag_num)) _tag_num)
  (:lane_detected
   (&optional __lane_detected)
   (if __lane_detected (setq _lane_detected __lane_detected)) _lane_detected)
  (:serialization-length
   ()
   (+
    ;; float32 _x_pos
    4
    ;; float32 _y_pos
    4
    ;; float32 _orientation
    4
    ;; bool _read_barcode
    1
    ;; int32 _tag_num
    4
    ;; int32 _lane_detected
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _x_pos
       (sys::poke _x_pos (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y_pos
       (sys::poke _y_pos (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _orientation
       (sys::poke _orientation (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _read_barcode
       (if _read_barcode (write-byte -1 s) (write-byte 0 s))
     ;; int32 _tag_num
       (write-long _tag_num s)
     ;; int32 _lane_detected
       (write-long _lane_detected s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _x_pos
     (setq _x_pos (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y_pos
     (setq _y_pos (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _orientation
     (setq _orientation (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _read_barcode
     (setq _read_barcode (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int32 _tag_num
     (setq _tag_num (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _lane_detected
     (setq _lane_detected (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get moobot_pgv::pgv_scan_data :md5sum-) "fb34df9bdee5103f0f83bb286c7a7b50")
(setf (get moobot_pgv::pgv_scan_data :datatype-) "moobot_pgv/pgv_scan_data")
(setf (get moobot_pgv::pgv_scan_data :definition-)
      "float32 x_pos
float32 y_pos
float32 orientation
bool read_barcode
int32 tag_num
int32 lane_detected #0 when lane_detected
")



(provide :moobot_pgv/pgv_scan_data "fb34df9bdee5103f0f83bb286c7a7b50")


