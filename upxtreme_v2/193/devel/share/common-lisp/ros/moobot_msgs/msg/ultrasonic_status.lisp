; Auto-generated. Do not edit!


(cl:in-package moobot_msgs-msg)


;//! \htmlinclude ultrasonic_status.msg.html

(cl:defclass <ultrasonic_status> (roslisp-msg-protocol:ros-message)
  ((S_Protection
    :reader S_Protection
    :initarg :S_Protection
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ultrasonic_status (<ultrasonic_status>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ultrasonic_status>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ultrasonic_status)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name moobot_msgs-msg:<ultrasonic_status> is deprecated: use moobot_msgs-msg:ultrasonic_status instead.")))

(cl:ensure-generic-function 'S_Protection-val :lambda-list '(m))
(cl:defmethod S_Protection-val ((m <ultrasonic_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_msgs-msg:S_Protection-val is deprecated.  Use moobot_msgs-msg:S_Protection instead.")
  (S_Protection m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ultrasonic_status>) ostream)
  "Serializes a message object of type '<ultrasonic_status>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'S_Protection) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ultrasonic_status>) istream)
  "Deserializes a message object of type '<ultrasonic_status>"
    (cl:setf (cl:slot-value msg 'S_Protection) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ultrasonic_status>)))
  "Returns string type for a message object of type '<ultrasonic_status>"
  "moobot_msgs/ultrasonic_status")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ultrasonic_status)))
  "Returns string type for a message object of type 'ultrasonic_status"
  "moobot_msgs/ultrasonic_status")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ultrasonic_status>)))
  "Returns md5sum for a message object of type '<ultrasonic_status>"
  "efb803211b7dc44878ac181a026e608e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ultrasonic_status)))
  "Returns md5sum for a message object of type 'ultrasonic_status"
  "efb803211b7dc44878ac181a026e608e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ultrasonic_status>)))
  "Returns full string definition for message of type '<ultrasonic_status>"
  (cl:format cl:nil "bool S_Protection~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ultrasonic_status)))
  "Returns full string definition for message of type 'ultrasonic_status"
  (cl:format cl:nil "bool S_Protection~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ultrasonic_status>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ultrasonic_status>))
  "Converts a ROS message object to a list"
  (cl:list 'ultrasonic_status
    (cl:cons ':S_Protection (S_Protection msg))
))
