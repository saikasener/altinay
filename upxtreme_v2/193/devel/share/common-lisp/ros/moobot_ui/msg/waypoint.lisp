; Auto-generated. Do not edit!


(cl:in-package moobot_ui-msg)


;//! \htmlinclude waypoint.msg.html

(cl:defclass <waypoint> (roslisp-msg-protocol:ros-message)
  ((way_point_name
    :reader way_point_name
    :initarg :way_point_name
    :type cl:string
    :initform "")
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

(cl:defclass waypoint (<waypoint>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <waypoint>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'waypoint)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name moobot_ui-msg:<waypoint> is deprecated: use moobot_ui-msg:waypoint instead.")))

(cl:ensure-generic-function 'way_point_name-val :lambda-list '(m))
(cl:defmethod way_point_name-val ((m <waypoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_ui-msg:way_point_name-val is deprecated.  Use moobot_ui-msg:way_point_name instead.")
  (way_point_name m))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <waypoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_ui-msg:x-val is deprecated.  Use moobot_ui-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <waypoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_ui-msg:y-val is deprecated.  Use moobot_ui-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <waypoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_ui-msg:z-val is deprecated.  Use moobot_ui-msg:z instead.")
  (z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <waypoint>) ostream)
  "Serializes a message object of type '<waypoint>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'way_point_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'way_point_name))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <waypoint>) istream)
  "Deserializes a message object of type '<waypoint>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'way_point_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'way_point_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<waypoint>)))
  "Returns string type for a message object of type '<waypoint>"
  "moobot_ui/waypoint")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'waypoint)))
  "Returns string type for a message object of type 'waypoint"
  "moobot_ui/waypoint")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<waypoint>)))
  "Returns md5sum for a message object of type '<waypoint>"
  "a43892a9a5e3559df1641a6a9d36bbe8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'waypoint)))
  "Returns md5sum for a message object of type 'waypoint"
  "a43892a9a5e3559df1641a6a9d36bbe8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<waypoint>)))
  "Returns full string definition for message of type '<waypoint>"
  (cl:format cl:nil "string way_point_name~%float32 x~%float32 y~%float32 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'waypoint)))
  "Returns full string definition for message of type 'waypoint"
  (cl:format cl:nil "string way_point_name~%float32 x~%float32 y~%float32 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <waypoint>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'way_point_name))
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <waypoint>))
  "Converts a ROS message object to a list"
  (cl:list 'waypoint
    (cl:cons ':way_point_name (way_point_name msg))
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
))
