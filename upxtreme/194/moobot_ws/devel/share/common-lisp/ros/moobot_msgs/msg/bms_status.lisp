; Auto-generated. Do not edit!


(cl:in-package moobot_msgs-msg)


;//! \htmlinclude bms_status.msg.html

(cl:defclass <bms_status> (roslisp-msg-protocol:ros-message)
  ((battery_volt_1
    :reader battery_volt_1
    :initarg :battery_volt_1
    :type cl:float
    :initform 0.0)
   (battery_volt_2
    :reader battery_volt_2
    :initarg :battery_volt_2
    :type cl:float
    :initform 0.0)
   (battery_volt_total
    :reader battery_volt_total
    :initarg :battery_volt_total
    :type cl:float
    :initform 0.0)
   (battery_temp_1
    :reader battery_temp_1
    :initarg :battery_temp_1
    :type cl:float
    :initform 0.0)
   (battery_temp_2
    :reader battery_temp_2
    :initarg :battery_temp_2
    :type cl:float
    :initform 0.0)
   (current_main
    :reader current_main
    :initarg :current_main
    :type cl:float
    :initform 0.0)
   (total_power_wh
    :reader total_power_wh
    :initarg :total_power_wh
    :type cl:float
    :initform 0.0))
)

(cl:defclass bms_status (<bms_status>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <bms_status>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'bms_status)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name moobot_msgs-msg:<bms_status> is deprecated: use moobot_msgs-msg:bms_status instead.")))

(cl:ensure-generic-function 'battery_volt_1-val :lambda-list '(m))
(cl:defmethod battery_volt_1-val ((m <bms_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_msgs-msg:battery_volt_1-val is deprecated.  Use moobot_msgs-msg:battery_volt_1 instead.")
  (battery_volt_1 m))

(cl:ensure-generic-function 'battery_volt_2-val :lambda-list '(m))
(cl:defmethod battery_volt_2-val ((m <bms_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_msgs-msg:battery_volt_2-val is deprecated.  Use moobot_msgs-msg:battery_volt_2 instead.")
  (battery_volt_2 m))

(cl:ensure-generic-function 'battery_volt_total-val :lambda-list '(m))
(cl:defmethod battery_volt_total-val ((m <bms_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_msgs-msg:battery_volt_total-val is deprecated.  Use moobot_msgs-msg:battery_volt_total instead.")
  (battery_volt_total m))

(cl:ensure-generic-function 'battery_temp_1-val :lambda-list '(m))
(cl:defmethod battery_temp_1-val ((m <bms_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_msgs-msg:battery_temp_1-val is deprecated.  Use moobot_msgs-msg:battery_temp_1 instead.")
  (battery_temp_1 m))

(cl:ensure-generic-function 'battery_temp_2-val :lambda-list '(m))
(cl:defmethod battery_temp_2-val ((m <bms_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_msgs-msg:battery_temp_2-val is deprecated.  Use moobot_msgs-msg:battery_temp_2 instead.")
  (battery_temp_2 m))

(cl:ensure-generic-function 'current_main-val :lambda-list '(m))
(cl:defmethod current_main-val ((m <bms_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_msgs-msg:current_main-val is deprecated.  Use moobot_msgs-msg:current_main instead.")
  (current_main m))

(cl:ensure-generic-function 'total_power_wh-val :lambda-list '(m))
(cl:defmethod total_power_wh-val ((m <bms_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_msgs-msg:total_power_wh-val is deprecated.  Use moobot_msgs-msg:total_power_wh instead.")
  (total_power_wh m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <bms_status>) ostream)
  "Serializes a message object of type '<bms_status>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'battery_volt_1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'battery_volt_2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'battery_volt_total))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'battery_temp_1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'battery_temp_2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'current_main))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'total_power_wh))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <bms_status>) istream)
  "Deserializes a message object of type '<bms_status>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'battery_volt_1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'battery_volt_2) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'battery_volt_total) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'battery_temp_1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'battery_temp_2) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current_main) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'total_power_wh) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<bms_status>)))
  "Returns string type for a message object of type '<bms_status>"
  "moobot_msgs/bms_status")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'bms_status)))
  "Returns string type for a message object of type 'bms_status"
  "moobot_msgs/bms_status")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<bms_status>)))
  "Returns md5sum for a message object of type '<bms_status>"
  "3113256c57eafd1210064dbad908f61d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'bms_status)))
  "Returns md5sum for a message object of type 'bms_status"
  "3113256c57eafd1210064dbad908f61d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<bms_status>)))
  "Returns full string definition for message of type '<bms_status>"
  (cl:format cl:nil "float32 battery_volt_1~%float32 battery_volt_2~%float32 battery_volt_total~%float32 battery_temp_1~%float32 battery_temp_2~%float32 current_main~%float32 total_power_wh~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'bms_status)))
  "Returns full string definition for message of type 'bms_status"
  (cl:format cl:nil "float32 battery_volt_1~%float32 battery_volt_2~%float32 battery_volt_total~%float32 battery_temp_1~%float32 battery_temp_2~%float32 current_main~%float32 total_power_wh~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <bms_status>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <bms_status>))
  "Converts a ROS message object to a list"
  (cl:list 'bms_status
    (cl:cons ':battery_volt_1 (battery_volt_1 msg))
    (cl:cons ':battery_volt_2 (battery_volt_2 msg))
    (cl:cons ':battery_volt_total (battery_volt_total msg))
    (cl:cons ':battery_temp_1 (battery_temp_1 msg))
    (cl:cons ':battery_temp_2 (battery_temp_2 msg))
    (cl:cons ':current_main (current_main msg))
    (cl:cons ':total_power_wh (total_power_wh msg))
))
