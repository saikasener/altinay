; Auto-generated. Do not edit!


(cl:in-package moobot_ui-msg)


;//! \htmlinclude pid_msg.msg.html

(cl:defclass <pid_msg> (roslisp-msg-protocol:ros-message)
  ((kp
    :reader kp
    :initarg :kp
    :type cl:float
    :initform 0.0)
   (ki
    :reader ki
    :initarg :ki
    :type cl:float
    :initform 0.0)
   (kd
    :reader kd
    :initarg :kd
    :type cl:float
    :initform 0.0))
)

(cl:defclass pid_msg (<pid_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <pid_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'pid_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name moobot_ui-msg:<pid_msg> is deprecated: use moobot_ui-msg:pid_msg instead.")))

(cl:ensure-generic-function 'kp-val :lambda-list '(m))
(cl:defmethod kp-val ((m <pid_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_ui-msg:kp-val is deprecated.  Use moobot_ui-msg:kp instead.")
  (kp m))

(cl:ensure-generic-function 'ki-val :lambda-list '(m))
(cl:defmethod ki-val ((m <pid_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_ui-msg:ki-val is deprecated.  Use moobot_ui-msg:ki instead.")
  (ki m))

(cl:ensure-generic-function 'kd-val :lambda-list '(m))
(cl:defmethod kd-val ((m <pid_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_ui-msg:kd-val is deprecated.  Use moobot_ui-msg:kd instead.")
  (kd m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <pid_msg>) ostream)
  "Serializes a message object of type '<pid_msg>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'kp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ki))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'kd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <pid_msg>) istream)
  "Deserializes a message object of type '<pid_msg>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'kp) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ki) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'kd) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<pid_msg>)))
  "Returns string type for a message object of type '<pid_msg>"
  "moobot_ui/pid_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'pid_msg)))
  "Returns string type for a message object of type 'pid_msg"
  "moobot_ui/pid_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<pid_msg>)))
  "Returns md5sum for a message object of type '<pid_msg>"
  "08d0ca1f582560895ecba909de1d88ec")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'pid_msg)))
  "Returns md5sum for a message object of type 'pid_msg"
  "08d0ca1f582560895ecba909de1d88ec")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<pid_msg>)))
  "Returns full string definition for message of type '<pid_msg>"
  (cl:format cl:nil "float32 kp~%float32 ki~%float32 kd~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'pid_msg)))
  "Returns full string definition for message of type 'pid_msg"
  (cl:format cl:nil "float32 kp~%float32 ki~%float32 kd~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <pid_msg>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <pid_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'pid_msg
    (cl:cons ':kp (kp msg))
    (cl:cons ':ki (ki msg))
    (cl:cons ':kd (kd msg))
))
