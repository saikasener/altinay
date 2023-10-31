; Auto-generated. Do not edit!


(cl:in-package moobot_msgs-msg)


;//! \htmlinclude moobot_sensor_status.msg.html

(cl:defclass <moobot_sensor_status> (roslisp-msg-protocol:ros-message)
  ((state
    :reader state
    :initarg :state
    :type cl:string
    :initform "")
   (state_type
    :reader state_type
    :initarg :state_type
    :type cl:fixnum
    :initform 0))
)

(cl:defclass moobot_sensor_status (<moobot_sensor_status>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <moobot_sensor_status>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'moobot_sensor_status)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name moobot_msgs-msg:<moobot_sensor_status> is deprecated: use moobot_msgs-msg:moobot_sensor_status instead.")))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <moobot_sensor_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_msgs-msg:state-val is deprecated.  Use moobot_msgs-msg:state instead.")
  (state m))

(cl:ensure-generic-function 'state_type-val :lambda-list '(m))
(cl:defmethod state_type-val ((m <moobot_sensor_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_msgs-msg:state_type-val is deprecated.  Use moobot_msgs-msg:state_type instead.")
  (state_type m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <moobot_sensor_status>) ostream)
  "Serializes a message object of type '<moobot_sensor_status>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'state))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'state))
  (cl:let* ((signed (cl:slot-value msg 'state_type)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <moobot_sensor_status>) istream)
  "Deserializes a message object of type '<moobot_sensor_status>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'state) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'state) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'state_type) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<moobot_sensor_status>)))
  "Returns string type for a message object of type '<moobot_sensor_status>"
  "moobot_msgs/moobot_sensor_status")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'moobot_sensor_status)))
  "Returns string type for a message object of type 'moobot_sensor_status"
  "moobot_msgs/moobot_sensor_status")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<moobot_sensor_status>)))
  "Returns md5sum for a message object of type '<moobot_sensor_status>"
  "49c3408f94118958df8cb878e67805f5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'moobot_sensor_status)))
  "Returns md5sum for a message object of type 'moobot_sensor_status"
  "49c3408f94118958df8cb878e67805f5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<moobot_sensor_status>)))
  "Returns full string definition for message of type '<moobot_sensor_status>"
  (cl:format cl:nil "string state~%int16 state_type~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'moobot_sensor_status)))
  "Returns full string definition for message of type 'moobot_sensor_status"
  (cl:format cl:nil "string state~%int16 state_type~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <moobot_sensor_status>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'state))
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <moobot_sensor_status>))
  "Converts a ROS message object to a list"
  (cl:list 'moobot_sensor_status
    (cl:cons ':state (state msg))
    (cl:cons ':state_type (state_type msg))
))
