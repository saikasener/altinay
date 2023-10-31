; Auto-generated. Do not edit!


(cl:in-package moobot_msgs-msg)


;//! \htmlinclude SensorState.msg.html

(cl:defclass <SensorState> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (led
    :reader led
    :initarg :led
    :type cl:fixnum
    :initform 0)
   (left_encoder
    :reader left_encoder
    :initarg :left_encoder
    :type cl:integer
    :initform 0)
   (right_encoder
    :reader right_encoder
    :initarg :right_encoder
    :type cl:integer
    :initform 0)
   (battery
    :reader battery
    :initarg :battery
    :type cl:float
    :initform 0.0))
)

(cl:defclass SensorState (<SensorState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SensorState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SensorState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name moobot_msgs-msg:<SensorState> is deprecated: use moobot_msgs-msg:SensorState instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <SensorState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_msgs-msg:header-val is deprecated.  Use moobot_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'led-val :lambda-list '(m))
(cl:defmethod led-val ((m <SensorState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_msgs-msg:led-val is deprecated.  Use moobot_msgs-msg:led instead.")
  (led m))

(cl:ensure-generic-function 'left_encoder-val :lambda-list '(m))
(cl:defmethod left_encoder-val ((m <SensorState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_msgs-msg:left_encoder-val is deprecated.  Use moobot_msgs-msg:left_encoder instead.")
  (left_encoder m))

(cl:ensure-generic-function 'right_encoder-val :lambda-list '(m))
(cl:defmethod right_encoder-val ((m <SensorState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_msgs-msg:right_encoder-val is deprecated.  Use moobot_msgs-msg:right_encoder instead.")
  (right_encoder m))

(cl:ensure-generic-function 'battery-val :lambda-list '(m))
(cl:defmethod battery-val ((m <SensorState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_msgs-msg:battery-val is deprecated.  Use moobot_msgs-msg:battery instead.")
  (battery m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<SensorState>)))
    "Constants for message type '<SensorState>"
  '((:ERROR_LEFT_MOTOR . 1)
    (:ERROR_RIGHT_MOTOR . 2)
    (:TORQUE_ON . 1)
    (:TORQUE_OFF . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'SensorState)))
    "Constants for message type 'SensorState"
  '((:ERROR_LEFT_MOTOR . 1)
    (:ERROR_RIGHT_MOTOR . 2)
    (:TORQUE_ON . 1)
    (:TORQUE_OFF . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SensorState>) ostream)
  "Serializes a message object of type '<SensorState>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'led)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'left_encoder)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'right_encoder)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'battery))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SensorState>) istream)
  "Deserializes a message object of type '<SensorState>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'led)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'left_encoder) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'right_encoder) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'battery) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SensorState>)))
  "Returns string type for a message object of type '<SensorState>"
  "moobot_msgs/SensorState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SensorState)))
  "Returns string type for a message object of type 'SensorState"
  "moobot_msgs/SensorState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SensorState>)))
  "Returns md5sum for a message object of type '<SensorState>"
  "84b397cf66be520b2eaf2f8ecf9e38be")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SensorState)))
  "Returns md5sum for a message object of type 'SensorState"
  "84b397cf66be520b2eaf2f8ecf9e38be")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SensorState>)))
  "Returns full string definition for message of type '<SensorState>"
  (cl:format cl:nil "# Motor errors~%uint8 ERROR_LEFT_MOTOR  = 1~%uint8 ERROR_RIGHT_MOTOR = 2~%~%# Motor torque~%uint8 TORQUE_ON  = 1~%uint8 TORQUE_OFF = 2~%~%########################################~%# Messages~%########################################~%Header  header~%uint8   led~%~%int64  left_encoder    # (-2,147,483,648 ~~ 2,147,483,647)~%int64  right_encoder   # (-2,147,483,648 ~~ 2,147,483,647)~%#bool   torque~%float32  battery~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SensorState)))
  "Returns full string definition for message of type 'SensorState"
  (cl:format cl:nil "# Motor errors~%uint8 ERROR_LEFT_MOTOR  = 1~%uint8 ERROR_RIGHT_MOTOR = 2~%~%# Motor torque~%uint8 TORQUE_ON  = 1~%uint8 TORQUE_OFF = 2~%~%########################################~%# Messages~%########################################~%Header  header~%uint8   led~%~%int64  left_encoder    # (-2,147,483,648 ~~ 2,147,483,647)~%int64  right_encoder   # (-2,147,483,648 ~~ 2,147,483,647)~%#bool   torque~%float32  battery~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SensorState>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     8
     8
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SensorState>))
  "Converts a ROS message object to a list"
  (cl:list 'SensorState
    (cl:cons ':header (header msg))
    (cl:cons ':led (led msg))
    (cl:cons ':left_encoder (left_encoder msg))
    (cl:cons ':right_encoder (right_encoder msg))
    (cl:cons ':battery (battery msg))
))
