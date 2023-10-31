; Auto-generated. Do not edit!


(cl:in-package moobot_msgs-msg)


;//! \htmlinclude moobot_status.msg.html

(cl:defclass <moobot_status> (roslisp-msg-protocol:ros-message)
  ((agv_mode
    :reader agv_mode
    :initarg :agv_mode
    :type cl:integer
    :initform 0)
   (agv_stopped
    :reader agv_stopped
    :initarg :agv_stopped
    :type cl:boolean
    :initform cl:nil)
   (emergency
    :reader emergency
    :initarg :emergency
    :type cl:boolean
    :initform cl:nil)
   (fault
    :reader fault
    :initarg :fault
    :type cl:boolean
    :initform cl:nil)
   (job
    :reader job
    :initarg :job
    :type cl:string
    :initform ""))
)

(cl:defclass moobot_status (<moobot_status>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <moobot_status>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'moobot_status)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name moobot_msgs-msg:<moobot_status> is deprecated: use moobot_msgs-msg:moobot_status instead.")))

(cl:ensure-generic-function 'agv_mode-val :lambda-list '(m))
(cl:defmethod agv_mode-val ((m <moobot_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_msgs-msg:agv_mode-val is deprecated.  Use moobot_msgs-msg:agv_mode instead.")
  (agv_mode m))

(cl:ensure-generic-function 'agv_stopped-val :lambda-list '(m))
(cl:defmethod agv_stopped-val ((m <moobot_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_msgs-msg:agv_stopped-val is deprecated.  Use moobot_msgs-msg:agv_stopped instead.")
  (agv_stopped m))

(cl:ensure-generic-function 'emergency-val :lambda-list '(m))
(cl:defmethod emergency-val ((m <moobot_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_msgs-msg:emergency-val is deprecated.  Use moobot_msgs-msg:emergency instead.")
  (emergency m))

(cl:ensure-generic-function 'fault-val :lambda-list '(m))
(cl:defmethod fault-val ((m <moobot_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_msgs-msg:fault-val is deprecated.  Use moobot_msgs-msg:fault instead.")
  (fault m))

(cl:ensure-generic-function 'job-val :lambda-list '(m))
(cl:defmethod job-val ((m <moobot_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_msgs-msg:job-val is deprecated.  Use moobot_msgs-msg:job instead.")
  (job m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <moobot_status>) ostream)
  "Serializes a message object of type '<moobot_status>"
  (cl:let* ((signed (cl:slot-value msg 'agv_mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'agv_stopped) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'emergency) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'fault) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'job))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'job))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <moobot_status>) istream)
  "Deserializes a message object of type '<moobot_status>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'agv_mode) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:slot-value msg 'agv_stopped) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'emergency) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'fault) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'job) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'job) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<moobot_status>)))
  "Returns string type for a message object of type '<moobot_status>"
  "moobot_msgs/moobot_status")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'moobot_status)))
  "Returns string type for a message object of type 'moobot_status"
  "moobot_msgs/moobot_status")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<moobot_status>)))
  "Returns md5sum for a message object of type '<moobot_status>"
  "6ac369f66a5b4dcf60abbe45c6d5c34f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'moobot_status)))
  "Returns md5sum for a message object of type 'moobot_status"
  "6ac369f66a5b4dcf60abbe45c6d5c34f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<moobot_status>)))
  "Returns full string definition for message of type '<moobot_status>"
  (cl:format cl:nil "int32 agv_mode~%bool agv_stopped~%bool emergency~%bool fault~%string job~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'moobot_status)))
  "Returns full string definition for message of type 'moobot_status"
  (cl:format cl:nil "int32 agv_mode~%bool agv_stopped~%bool emergency~%bool fault~%string job~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <moobot_status>))
  (cl:+ 0
     4
     1
     1
     1
     4 (cl:length (cl:slot-value msg 'job))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <moobot_status>))
  "Converts a ROS message object to a list"
  (cl:list 'moobot_status
    (cl:cons ':agv_mode (agv_mode msg))
    (cl:cons ':agv_stopped (agv_stopped msg))
    (cl:cons ':emergency (emergency msg))
    (cl:cons ':fault (fault msg))
    (cl:cons ':job (job msg))
))
