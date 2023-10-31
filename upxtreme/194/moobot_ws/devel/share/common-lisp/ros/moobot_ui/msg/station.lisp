; Auto-generated. Do not edit!


(cl:in-package moobot_ui-msg)


;//! \htmlinclude station.msg.html

(cl:defclass <station> (roslisp-msg-protocol:ros-message)
  ((station_number
    :reader station_number
    :initarg :station_number
    :type cl:integer
    :initform 0)
   (x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (z
    :reader z
    :initarg :z
    :type cl:float
    :initform 0.0))
)

(cl:defclass station (<station>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <station>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'station)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name moobot_ui-msg:<station> is deprecated: use moobot_ui-msg:station instead.")))

(cl:ensure-generic-function 'station_number-val :lambda-list '(m))
(cl:defmethod station_number-val ((m <station>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_ui-msg:station_number-val is deprecated.  Use moobot_ui-msg:station_number instead.")
  (station_number m))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <station>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_ui-msg:x-val is deprecated.  Use moobot_ui-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <station>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_ui-msg:y-val is deprecated.  Use moobot_ui-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <station>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_ui-msg:z-val is deprecated.  Use moobot_ui-msg:z instead.")
  (z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <station>) ostream)
  "Serializes a message object of type '<station>"
  (cl:let* ((signed (cl:slot-value msg 'station_number)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <station>) istream)
  "Deserializes a message object of type '<station>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'station_number) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<station>)))
  "Returns string type for a message object of type '<station>"
  "moobot_ui/station")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'station)))
  "Returns string type for a message object of type 'station"
  "moobot_ui/station")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<station>)))
  "Returns md5sum for a message object of type '<station>"
  "5c4f6212c6e666a3f4185ee1f58383a2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'station)))
  "Returns md5sum for a message object of type 'station"
  "5c4f6212c6e666a3f4185ee1f58383a2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<station>)))
  "Returns full string definition for message of type '<station>"
  (cl:format cl:nil "int32 station_number~%float32 x~%float32 y~%float32 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'station)))
  "Returns full string definition for message of type 'station"
  (cl:format cl:nil "int32 station_number~%float32 x~%float32 y~%float32 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <station>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <station>))
  "Converts a ROS message object to a list"
  (cl:list 'station
    (cl:cons ':station_number (station_number msg))
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
))
