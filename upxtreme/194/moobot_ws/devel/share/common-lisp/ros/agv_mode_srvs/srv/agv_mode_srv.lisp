; Auto-generated. Do not edit!


(cl:in-package agv_mode_srvs-srv)


;//! \htmlinclude agv_mode_srv-request.msg.html

(cl:defclass <agv_mode_srv-request> (roslisp-msg-protocol:ros-message)
  ((mode
    :reader mode
    :initarg :mode
    :type cl:integer
    :initform 0))
)

(cl:defclass agv_mode_srv-request (<agv_mode_srv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <agv_mode_srv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'agv_mode_srv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name agv_mode_srvs-srv:<agv_mode_srv-request> is deprecated: use agv_mode_srvs-srv:agv_mode_srv-request instead.")))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <agv_mode_srv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader agv_mode_srvs-srv:mode-val is deprecated.  Use agv_mode_srvs-srv:mode instead.")
  (mode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <agv_mode_srv-request>) ostream)
  "Serializes a message object of type '<agv_mode_srv-request>"
  (cl:let* ((signed (cl:slot-value msg 'mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <agv_mode_srv-request>) istream)
  "Deserializes a message object of type '<agv_mode_srv-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mode) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<agv_mode_srv-request>)))
  "Returns string type for a service object of type '<agv_mode_srv-request>"
  "agv_mode_srvs/agv_mode_srvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'agv_mode_srv-request)))
  "Returns string type for a service object of type 'agv_mode_srv-request"
  "agv_mode_srvs/agv_mode_srvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<agv_mode_srv-request>)))
  "Returns md5sum for a message object of type '<agv_mode_srv-request>"
  "ae96190e987c19063498c13c3886f347")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'agv_mode_srv-request)))
  "Returns md5sum for a message object of type 'agv_mode_srv-request"
  "ae96190e987c19063498c13c3886f347")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<agv_mode_srv-request>)))
  "Returns full string definition for message of type '<agv_mode_srv-request>"
  (cl:format cl:nil "int64 mode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'agv_mode_srv-request)))
  "Returns full string definition for message of type 'agv_mode_srv-request"
  (cl:format cl:nil "int64 mode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <agv_mode_srv-request>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <agv_mode_srv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'agv_mode_srv-request
    (cl:cons ':mode (mode msg))
))
;//! \htmlinclude agv_mode_srv-response.msg.html

(cl:defclass <agv_mode_srv-response> (roslisp-msg-protocol:ros-message)
  ((newmode
    :reader newmode
    :initarg :newmode
    :type cl:integer
    :initform 0))
)

(cl:defclass agv_mode_srv-response (<agv_mode_srv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <agv_mode_srv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'agv_mode_srv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name agv_mode_srvs-srv:<agv_mode_srv-response> is deprecated: use agv_mode_srvs-srv:agv_mode_srv-response instead.")))

(cl:ensure-generic-function 'newmode-val :lambda-list '(m))
(cl:defmethod newmode-val ((m <agv_mode_srv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader agv_mode_srvs-srv:newmode-val is deprecated.  Use agv_mode_srvs-srv:newmode instead.")
  (newmode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <agv_mode_srv-response>) ostream)
  "Serializes a message object of type '<agv_mode_srv-response>"
  (cl:let* ((signed (cl:slot-value msg 'newmode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <agv_mode_srv-response>) istream)
  "Deserializes a message object of type '<agv_mode_srv-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'newmode) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<agv_mode_srv-response>)))
  "Returns string type for a service object of type '<agv_mode_srv-response>"
  "agv_mode_srvs/agv_mode_srvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'agv_mode_srv-response)))
  "Returns string type for a service object of type 'agv_mode_srv-response"
  "agv_mode_srvs/agv_mode_srvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<agv_mode_srv-response>)))
  "Returns md5sum for a message object of type '<agv_mode_srv-response>"
  "ae96190e987c19063498c13c3886f347")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'agv_mode_srv-response)))
  "Returns md5sum for a message object of type 'agv_mode_srv-response"
  "ae96190e987c19063498c13c3886f347")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<agv_mode_srv-response>)))
  "Returns full string definition for message of type '<agv_mode_srv-response>"
  (cl:format cl:nil "int64 newmode~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'agv_mode_srv-response)))
  "Returns full string definition for message of type 'agv_mode_srv-response"
  (cl:format cl:nil "int64 newmode~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <agv_mode_srv-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <agv_mode_srv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'agv_mode_srv-response
    (cl:cons ':newmode (newmode msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'agv_mode_srv)))
  'agv_mode_srv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'agv_mode_srv)))
  'agv_mode_srv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'agv_mode_srv)))
  "Returns string type for a service object of type '<agv_mode_srv>"
  "agv_mode_srvs/agv_mode_srv")