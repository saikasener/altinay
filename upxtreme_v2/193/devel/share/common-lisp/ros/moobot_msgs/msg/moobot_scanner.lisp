; Auto-generated. Do not edit!


(cl:in-package moobot_msgs-msg)


;//! \htmlinclude moobot_scanner.msg.html

(cl:defclass <moobot_scanner> (roslisp-msg-protocol:ros-message)
  ((fs_ossd
    :reader fs_ossd
    :initarg :fs_ossd
    :type cl:integer
    :initform 0)
   (fs_w1
    :reader fs_w1
    :initarg :fs_w1
    :type cl:integer
    :initform 0)
   (fs_w2
    :reader fs_w2
    :initarg :fs_w2
    :type cl:integer
    :initform 0)
   (bs_ossd
    :reader bs_ossd
    :initarg :bs_ossd
    :type cl:integer
    :initform 0)
   (bs_w1
    :reader bs_w1
    :initarg :bs_w1
    :type cl:integer
    :initform 0)
   (bs_w2
    :reader bs_w2
    :initarg :bs_w2
    :type cl:integer
    :initform 0))
)

(cl:defclass moobot_scanner (<moobot_scanner>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <moobot_scanner>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'moobot_scanner)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name moobot_msgs-msg:<moobot_scanner> is deprecated: use moobot_msgs-msg:moobot_scanner instead.")))

(cl:ensure-generic-function 'fs_ossd-val :lambda-list '(m))
(cl:defmethod fs_ossd-val ((m <moobot_scanner>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_msgs-msg:fs_ossd-val is deprecated.  Use moobot_msgs-msg:fs_ossd instead.")
  (fs_ossd m))

(cl:ensure-generic-function 'fs_w1-val :lambda-list '(m))
(cl:defmethod fs_w1-val ((m <moobot_scanner>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_msgs-msg:fs_w1-val is deprecated.  Use moobot_msgs-msg:fs_w1 instead.")
  (fs_w1 m))

(cl:ensure-generic-function 'fs_w2-val :lambda-list '(m))
(cl:defmethod fs_w2-val ((m <moobot_scanner>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_msgs-msg:fs_w2-val is deprecated.  Use moobot_msgs-msg:fs_w2 instead.")
  (fs_w2 m))

(cl:ensure-generic-function 'bs_ossd-val :lambda-list '(m))
(cl:defmethod bs_ossd-val ((m <moobot_scanner>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_msgs-msg:bs_ossd-val is deprecated.  Use moobot_msgs-msg:bs_ossd instead.")
  (bs_ossd m))

(cl:ensure-generic-function 'bs_w1-val :lambda-list '(m))
(cl:defmethod bs_w1-val ((m <moobot_scanner>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_msgs-msg:bs_w1-val is deprecated.  Use moobot_msgs-msg:bs_w1 instead.")
  (bs_w1 m))

(cl:ensure-generic-function 'bs_w2-val :lambda-list '(m))
(cl:defmethod bs_w2-val ((m <moobot_scanner>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_msgs-msg:bs_w2-val is deprecated.  Use moobot_msgs-msg:bs_w2 instead.")
  (bs_w2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <moobot_scanner>) ostream)
  "Serializes a message object of type '<moobot_scanner>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'fs_ossd)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'fs_w1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'fs_w2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'bs_ossd)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'bs_w1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'bs_w2)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <moobot_scanner>) istream)
  "Deserializes a message object of type '<moobot_scanner>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'fs_ossd)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'fs_w1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'fs_w2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'bs_ossd)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'bs_w1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'bs_w2)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<moobot_scanner>)))
  "Returns string type for a message object of type '<moobot_scanner>"
  "moobot_msgs/moobot_scanner")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'moobot_scanner)))
  "Returns string type for a message object of type 'moobot_scanner"
  "moobot_msgs/moobot_scanner")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<moobot_scanner>)))
  "Returns md5sum for a message object of type '<moobot_scanner>"
  "b971113dac55cb95f6b07a91c6331c49")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'moobot_scanner)))
  "Returns md5sum for a message object of type 'moobot_scanner"
  "b971113dac55cb95f6b07a91c6331c49")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<moobot_scanner>)))
  "Returns full string definition for message of type '<moobot_scanner>"
  (cl:format cl:nil "byte fs_ossd~%byte fs_w1~%byte fs_w2~%byte bs_ossd~%byte bs_w1~%byte bs_w2~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'moobot_scanner)))
  "Returns full string definition for message of type 'moobot_scanner"
  (cl:format cl:nil "byte fs_ossd~%byte fs_w1~%byte fs_w2~%byte bs_ossd~%byte bs_w1~%byte bs_w2~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <moobot_scanner>))
  (cl:+ 0
     1
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <moobot_scanner>))
  "Converts a ROS message object to a list"
  (cl:list 'moobot_scanner
    (cl:cons ':fs_ossd (fs_ossd msg))
    (cl:cons ':fs_w1 (fs_w1 msg))
    (cl:cons ':fs_w2 (fs_w2 msg))
    (cl:cons ':bs_ossd (bs_ossd msg))
    (cl:cons ':bs_w1 (bs_w1 msg))
    (cl:cons ':bs_w2 (bs_w2 msg))
))
