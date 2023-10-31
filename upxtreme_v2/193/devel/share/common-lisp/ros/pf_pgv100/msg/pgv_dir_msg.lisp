; Auto-generated. Do not edit!


(cl:in-package pf_pgv100-msg)


;//! \htmlinclude pgv_dir_msg.msg.html

(cl:defclass <pgv_dir_msg> (roslisp-msg-protocol:ros-message)
  ((dir_command
    :reader dir_command
    :initarg :dir_command
    :type cl:fixnum
    :initform 0))
)

(cl:defclass pgv_dir_msg (<pgv_dir_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <pgv_dir_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'pgv_dir_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pf_pgv100-msg:<pgv_dir_msg> is deprecated: use pf_pgv100-msg:pgv_dir_msg instead.")))

(cl:ensure-generic-function 'dir_command-val :lambda-list '(m))
(cl:defmethod dir_command-val ((m <pgv_dir_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pf_pgv100-msg:dir_command-val is deprecated.  Use pf_pgv100-msg:dir_command instead.")
  (dir_command m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <pgv_dir_msg>) ostream)
  "Serializes a message object of type '<pgv_dir_msg>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dir_command)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <pgv_dir_msg>) istream)
  "Deserializes a message object of type '<pgv_dir_msg>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dir_command)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<pgv_dir_msg>)))
  "Returns string type for a message object of type '<pgv_dir_msg>"
  "pf_pgv100/pgv_dir_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'pgv_dir_msg)))
  "Returns string type for a message object of type 'pgv_dir_msg"
  "pf_pgv100/pgv_dir_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<pgv_dir_msg>)))
  "Returns md5sum for a message object of type '<pgv_dir_msg>"
  "be420e78d04d79ee4d8fd23e20966af0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'pgv_dir_msg)))
  "Returns md5sum for a message object of type 'pgv_dir_msg"
  "be420e78d04d79ee4d8fd23e20966af0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<pgv_dir_msg>)))
  "Returns full string definition for message of type '<pgv_dir_msg>"
  (cl:format cl:nil "uint8 dir_command~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'pgv_dir_msg)))
  "Returns full string definition for message of type 'pgv_dir_msg"
  (cl:format cl:nil "uint8 dir_command~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <pgv_dir_msg>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <pgv_dir_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'pgv_dir_msg
    (cl:cons ':dir_command (dir_command msg))
))
