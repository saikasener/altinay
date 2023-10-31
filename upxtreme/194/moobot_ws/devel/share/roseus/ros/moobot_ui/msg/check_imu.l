;; Auto-generated. Do not edit!


(when (boundp 'moobot_ui::check_imu)
  (if (not (find-package "MOOBOT_UI"))
    (make-package "MOOBOT_UI"))
  (shadow 'check_imu (find-package "MOOBOT_UI")))
(unless (find-package "MOOBOT_UI::CHECK_IMU")
  (make-package "MOOBOT_UI::CHECK_IMU"))

(in-package "ROS")
;;//! \htmlinclude check_imu.msg.html


(defclass moobot_ui::check_imu
  :super ros::object
  :slots (_angular_vel_z _linear_acc_x ))

(defmethod moobot_ui::check_imu
  (:init
   (&key
    ((:angular_vel_z __angular_vel_z) 0.0)
    ((:linear_acc_x __linear_acc_x) 0.0)
    )
   (send-super :init)
   (setq _angular_vel_z (float __angular_vel_z))
   (setq _linear_acc_x (float __linear_acc_x))
   self)
  (:angular_vel_z
   (&optional __angular_vel_z)
   (if __angular_vel_z (setq _angular_vel_z __angular_vel_z)) _angular_vel_z)
  (:linear_acc_x
   (&optional __linear_acc_x)
   (if __linear_acc_x (setq _linear_acc_x __linear_acc_x)) _linear_acc_x)
  (:serialization-length
   ()
   (+
    ;; float32 _angular_vel_z
    4
    ;; float32 _linear_acc_x
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _angular_vel_z
       (sys::poke _angular_vel_z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _linear_acc_x
       (sys::poke _linear_acc_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _angular_vel_z
     (setq _angular_vel_z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _linear_acc_x
     (setq _linear_acc_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get moobot_ui::check_imu :md5sum-) "9b1b998330ad15a2d1903c80d31c89e4")
(setf (get moobot_ui::check_imu :datatype-) "moobot_ui/check_imu")
(setf (get moobot_ui::check_imu :definition-)
      "float32 angular_vel_z
float32 linear_acc_x

")



(provide :moobot_ui/check_imu "9b1b998330ad15a2d1903c80d31c89e4")


