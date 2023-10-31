; Auto-generated. Do not edit!


(cl:in-package moobot_msgs-msg)


;//! \htmlinclude conveyor_status.msg.html

(cl:defclass <conveyor_status> (roslisp-msg-protocol:ros-message)
  ((load_conveyor
    :reader load_conveyor
    :initarg :load_conveyor
    :type cl:boolean
    :initform cl:nil)
   (unload_conveyor
    :reader unload_conveyor
    :initarg :unload_conveyor
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass conveyor_status (<conveyor_status>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <conveyor_status>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'conveyor_status)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name moobot_msgs-msg:<conveyor_status> is deprecated: use moobot_msgs-msg:conveyor_status instead.")))

(cl:ensure-generic-function 'load_conveyor-val :lambda-list '(m))
(cl:defmethod load_conveyor-val ((m <conveyor_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_msgs-msg:load_conveyor-val is deprecated.  Use moobot_msgs-msg:load_conveyor instead.")
  (load_conveyor m))

(cl:ensure-generic-function 'unload_conveyor-val :lambda-list '(m))
(cl:defmethod unload_conveyor-val ((m <conveyor_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_msgs-msg:unload_conveyor-val is deprecated.  Use moobot_msgs-msg:unload_conveyor instead.")
  (unload_conveyor m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <conveyor_status>) ostream)
  "Serializes a message object of type '<conveyor_status>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'load_conveyor) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'unload_conveyor) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <conveyor_status>) istream)
  "Deserializes a message object of type '<conveyor_status>"
    (cl:setf (cl:slot-value msg 'load_conveyor) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'unload_conveyor) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<conveyor_status>)))
  "Returns string type for a message object of type '<conveyor_status>"
  "moobot_msgs/conveyor_status")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'conveyor_status)))
  "Returns string type for a message object of type 'conveyor_status"
  "moobot_msgs/conveyor_status")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<conveyor_status>)))
  "Returns md5sum for a message object of type '<conveyor_status>"
  "a8462790025e5d19ec5df8dbb71cb071")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'conveyor_status)))
  "Returns md5sum for a message object of type 'conveyor_status"
  "a8462790025e5d19ec5df8dbb71cb071")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<conveyor_status>)))
  "Returns full string definition for message of type '<conveyor_status>"
  (cl:format cl:nil "bool load_conveyor~%bool unload_conveyor~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'conveyor_status)))
  "Returns full string definition for message of type 'conveyor_status"
  (cl:format cl:nil "bool load_conveyor~%bool unload_conveyor~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <conveyor_status>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <conveyor_status>))
  "Converts a ROS message object to a list"
  (cl:list 'conveyor_status
    (cl:cons ':load_conveyor (load_conveyor msg))
    (cl:cons ':unload_conveyor (unload_conveyor msg))
))
