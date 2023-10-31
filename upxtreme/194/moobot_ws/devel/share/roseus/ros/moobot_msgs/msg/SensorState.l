;; Auto-generated. Do not edit!


(when (boundp 'moobot_msgs::SensorState)
  (if (not (find-package "MOOBOT_MSGS"))
    (make-package "MOOBOT_MSGS"))
  (shadow 'SensorState (find-package "MOOBOT_MSGS")))
(unless (find-package "MOOBOT_MSGS::SENSORSTATE")
  (make-package "MOOBOT_MSGS::SENSORSTATE"))

(in-package "ROS")
;;//! \htmlinclude SensorState.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*ERROR_LEFT_MOTOR*" (find-package "MOOBOT_MSGS::SENSORSTATE"))
(shadow '*ERROR_LEFT_MOTOR* (find-package "MOOBOT_MSGS::SENSORSTATE"))
(defconstant moobot_msgs::SensorState::*ERROR_LEFT_MOTOR* 1)
(intern "*ERROR_RIGHT_MOTOR*" (find-package "MOOBOT_MSGS::SENSORSTATE"))
(shadow '*ERROR_RIGHT_MOTOR* (find-package "MOOBOT_MSGS::SENSORSTATE"))
(defconstant moobot_msgs::SensorState::*ERROR_RIGHT_MOTOR* 2)
(intern "*TORQUE_ON*" (find-package "MOOBOT_MSGS::SENSORSTATE"))
(shadow '*TORQUE_ON* (find-package "MOOBOT_MSGS::SENSORSTATE"))
(defconstant moobot_msgs::SensorState::*TORQUE_ON* 1)
(intern "*TORQUE_OFF*" (find-package "MOOBOT_MSGS::SENSORSTATE"))
(shadow '*TORQUE_OFF* (find-package "MOOBOT_MSGS::SENSORSTATE"))
(defconstant moobot_msgs::SensorState::*TORQUE_OFF* 2)

(defun moobot_msgs::SensorState-to-symbol (const)
  (cond
        ((= const 1) 'moobot_msgs::SensorState::*ERROR_LEFT_MOTOR*)
        ((= const 2) 'moobot_msgs::SensorState::*ERROR_RIGHT_MOTOR*)
        ((= const 1) 'moobot_msgs::SensorState::*TORQUE_ON*)
        ((= const 2) 'moobot_msgs::SensorState::*TORQUE_OFF*)
        (t nil)))

(defclass moobot_msgs::SensorState
  :super ros::object
  :slots (_header _led _left_encoder _right_encoder _battery ))

(defmethod moobot_msgs::SensorState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:led __led) 0)
    ((:left_encoder __left_encoder) 0)
    ((:right_encoder __right_encoder) 0)
    ((:battery __battery) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _led (round __led))
   (setq _left_encoder (round __left_encoder))
   (setq _right_encoder (round __right_encoder))
   (setq _battery (float __battery))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:led
   (&optional __led)
   (if __led (setq _led __led)) _led)
  (:left_encoder
   (&optional __left_encoder)
   (if __left_encoder (setq _left_encoder __left_encoder)) _left_encoder)
  (:right_encoder
   (&optional __right_encoder)
   (if __right_encoder (setq _right_encoder __right_encoder)) _right_encoder)
  (:battery
   (&optional __battery)
   (if __battery (setq _battery __battery)) _battery)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _led
    1
    ;; int64 _left_encoder
    8
    ;; int64 _right_encoder
    8
    ;; float32 _battery
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _led
       (write-byte _led s)
     ;; int64 _left_encoder
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _left_encoder (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _left_encoder) (= (length (_left_encoder . bv)) 2)) ;; bignum
              (write-long (ash (elt (_left_encoder . bv) 0) 0) s)
              (write-long (ash (elt (_left_encoder . bv) 1) -1) s))
             ((and (class _left_encoder) (= (length (_left_encoder . bv)) 1)) ;; big1
              (write-long (elt (_left_encoder . bv) 0) s)
              (write-long (if (>= _left_encoder 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _left_encoder s)(write-long (if (>= _left_encoder 0) 0 #xffffffff) s)))
     ;; int64 _right_encoder
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _right_encoder (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _right_encoder) (= (length (_right_encoder . bv)) 2)) ;; bignum
              (write-long (ash (elt (_right_encoder . bv) 0) 0) s)
              (write-long (ash (elt (_right_encoder . bv) 1) -1) s))
             ((and (class _right_encoder) (= (length (_right_encoder . bv)) 1)) ;; big1
              (write-long (elt (_right_encoder . bv) 0) s)
              (write-long (if (>= _right_encoder 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _right_encoder s)(write-long (if (>= _right_encoder 0) 0 #xffffffff) s)))
     ;; float32 _battery
       (sys::poke _battery (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _led
     (setq _led (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; int64 _left_encoder
#+(or :alpha :irix6 :x86_64)
      (setf _left_encoder (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _left_encoder (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _right_encoder
#+(or :alpha :irix6 :x86_64)
      (setf _right_encoder (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _right_encoder (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; float32 _battery
     (setq _battery (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get moobot_msgs::SensorState :md5sum-) "84b397cf66be520b2eaf2f8ecf9e38be")
(setf (get moobot_msgs::SensorState :datatype-) "moobot_msgs/SensorState")
(setf (get moobot_msgs::SensorState :definition-)
      "# Motor errors
uint8 ERROR_LEFT_MOTOR  = 1
uint8 ERROR_RIGHT_MOTOR = 2

# Motor torque
uint8 TORQUE_ON  = 1
uint8 TORQUE_OFF = 2

########################################
# Messages
########################################
Header  header
uint8   led

int64  left_encoder    # (-2,147,483,648 ~ 2,147,483,647)
int64  right_encoder   # (-2,147,483,648 ~ 2,147,483,647)
#bool   torque
float32  battery

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



(provide :moobot_msgs/SensorState "84b397cf66be520b2eaf2f8ecf9e38be")


