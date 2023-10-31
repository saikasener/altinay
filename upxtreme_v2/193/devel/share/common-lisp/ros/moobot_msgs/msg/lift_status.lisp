; Auto-generated. Do not edit!


(cl:in-package moobot_msgs-msg)


;//! \htmlinclude lift_status.msg.html

(cl:defclass <lift_status> (roslisp-msg-protocol:ros-message)
  ((up_lift
    :reader up_lift
    :initarg :up_lift
    :type cl:boolean
    :initform cl:nil)
   (down_lift
    :reader down_lift
    :initarg :down_lift
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass lift_status (<lift_status>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <lift_status>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'lift_status)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name moobot_msgs-msg:<lift_status> is deprecated: use moobot_msgs-msg:lift_status instead.")))

(cl:ensure-generic-function 'up_lift-val :lambda-list '(m))
(cl:defmethod up_lift-val ((m <lift_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_msgs-msg:up_lift-val is deprecated.  Use moobot_msgs-msg:up_lift instead.")
  (up_lift m))

(cl:ensure-generic-function 'down_lift-val :lambda-list '(m))
(cl:defmethod down_lift-val ((m <lift_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_msgs-msg:down_lift-val is deprecated.  Use moobot_msgs-msg:down_lift instead.")
  (down_lift m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <lift_status>) ostream)
  "Serializes a message object of type '<lift_status>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'up_lift) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'down_lift) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <lift_status>) istream)
  "Deserializes a message object of type '<lift_status>"
    (cl:setf (cl:slot-value msg 'up_lift) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'down_lift) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<lift_status>)))
  "Returns string type for a message object of type '<lift_status>"
  "moobot_msgs/lift_status")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'lift_status)))
  "Returns string type for a message object of type 'lift_status"
  "moobot_msgs/lift_status")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<lift_status>)))
  "Returns md5sum for a message object of type '<lift_status>"
  "88d1b4abfd45857169d18e9ca11502ca")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'lift_status)))
  "Returns md5sum for a message object of type 'lift_status"
  "88d1b4abfd45857169d18e9ca11502ca")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<lift_status>)))
  "Returns full string definition for message of type '<lift_status>"
  (cl:format cl:nil "bool up_lift~%bool down_lift~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'lift_status)))
  "Returns full string definition for message of type 'lift_status"
  (cl:format cl:nil "bool up_lift~%bool down_lift~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <lift_status>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <lift_status>))
  "Converts a ROS message object to a list"
  (cl:list 'lift_status
    (cl:cons ':up_lift (up_lift msg))
    (cl:cons ':down_lift (down_lift msg))
))
