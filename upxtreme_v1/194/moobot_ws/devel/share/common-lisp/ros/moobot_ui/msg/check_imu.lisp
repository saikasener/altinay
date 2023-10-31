; Auto-generated. Do not edit!


(cl:in-package moobot_ui-msg)


;//! \htmlinclude check_imu.msg.html

(cl:defclass <check_imu> (roslisp-msg-protocol:ros-message)
  ((angular_vel_z
    :reader angular_vel_z
    :initarg :angular_vel_z
    :type cl:float
    :initform 0.0)
   (linear_acc_x
    :reader linear_acc_x
    :initarg :linear_acc_x
    :type cl:float
    :initform 0.0))
)

(cl:defclass check_imu (<check_imu>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <check_imu>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'check_imu)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name moobot_ui-msg:<check_imu> is deprecated: use moobot_ui-msg:check_imu instead.")))

(cl:ensure-generic-function 'angular_vel_z-val :lambda-list '(m))
(cl:defmethod angular_vel_z-val ((m <check_imu>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_ui-msg:angular_vel_z-val is deprecated.  Use moobot_ui-msg:angular_vel_z instead.")
  (angular_vel_z m))

(cl:ensure-generic-function 'linear_acc_x-val :lambda-list '(m))
(cl:defmethod linear_acc_x-val ((m <check_imu>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_ui-msg:linear_acc_x-val is deprecated.  Use moobot_ui-msg:linear_acc_x instead.")
  (linear_acc_x m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <check_imu>) ostream)
  "Serializes a message object of type '<check_imu>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angular_vel_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'linear_acc_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <check_imu>) istream)
  "Deserializes a message object of type '<check_imu>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angular_vel_z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'linear_acc_x) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<check_imu>)))
  "Returns string type for a message object of type '<check_imu>"
  "moobot_ui/check_imu")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'check_imu)))
  "Returns string type for a message object of type 'check_imu"
  "moobot_ui/check_imu")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<check_imu>)))
  "Returns md5sum for a message object of type '<check_imu>"
  "9b1b998330ad15a2d1903c80d31c89e4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'check_imu)))
  "Returns md5sum for a message object of type 'check_imu"
  "9b1b998330ad15a2d1903c80d31c89e4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<check_imu>)))
  "Returns full string definition for message of type '<check_imu>"
  (cl:format cl:nil "float32 angular_vel_z~%float32 linear_acc_x~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'check_imu)))
  "Returns full string definition for message of type 'check_imu"
  (cl:format cl:nil "float32 angular_vel_z~%float32 linear_acc_x~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <check_imu>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <check_imu>))
  "Converts a ROS message object to a list"
  (cl:list 'check_imu
    (cl:cons ':angular_vel_z (angular_vel_z msg))
    (cl:cons ':linear_acc_x (linear_acc_x msg))
))
