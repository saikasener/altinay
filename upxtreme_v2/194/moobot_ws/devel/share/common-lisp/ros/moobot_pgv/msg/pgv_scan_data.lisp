; Auto-generated. Do not edit!


(cl:in-package moobot_pgv-msg)


;//! \htmlinclude pgv_scan_data.msg.html

(cl:defclass <pgv_scan_data> (roslisp-msg-protocol:ros-message)
  ((x_pos
    :reader x_pos
    :initarg :x_pos
    :type cl:float
    :initform 0.0)
   (y_pos
    :reader y_pos
    :initarg :y_pos
    :type cl:float
    :initform 0.0)
   (orientation
    :reader orientation
    :initarg :orientation
    :type cl:float
    :initform 0.0)
   (read_barcode
    :reader read_barcode
    :initarg :read_barcode
    :type cl:boolean
    :initform cl:nil)
   (tag_num
    :reader tag_num
    :initarg :tag_num
    :type cl:integer
    :initform 0)
   (lane_detected
    :reader lane_detected
    :initarg :lane_detected
    :type cl:integer
    :initform 0))
)

(cl:defclass pgv_scan_data (<pgv_scan_data>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <pgv_scan_data>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'pgv_scan_data)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name moobot_pgv-msg:<pgv_scan_data> is deprecated: use moobot_pgv-msg:pgv_scan_data instead.")))

(cl:ensure-generic-function 'x_pos-val :lambda-list '(m))
(cl:defmethod x_pos-val ((m <pgv_scan_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_pgv-msg:x_pos-val is deprecated.  Use moobot_pgv-msg:x_pos instead.")
  (x_pos m))

(cl:ensure-generic-function 'y_pos-val :lambda-list '(m))
(cl:defmethod y_pos-val ((m <pgv_scan_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_pgv-msg:y_pos-val is deprecated.  Use moobot_pgv-msg:y_pos instead.")
  (y_pos m))

(cl:ensure-generic-function 'orientation-val :lambda-list '(m))
(cl:defmethod orientation-val ((m <pgv_scan_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_pgv-msg:orientation-val is deprecated.  Use moobot_pgv-msg:orientation instead.")
  (orientation m))

(cl:ensure-generic-function 'read_barcode-val :lambda-list '(m))
(cl:defmethod read_barcode-val ((m <pgv_scan_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_pgv-msg:read_barcode-val is deprecated.  Use moobot_pgv-msg:read_barcode instead.")
  (read_barcode m))

(cl:ensure-generic-function 'tag_num-val :lambda-list '(m))
(cl:defmethod tag_num-val ((m <pgv_scan_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_pgv-msg:tag_num-val is deprecated.  Use moobot_pgv-msg:tag_num instead.")
  (tag_num m))

(cl:ensure-generic-function 'lane_detected-val :lambda-list '(m))
(cl:defmethod lane_detected-val ((m <pgv_scan_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_pgv-msg:lane_detected-val is deprecated.  Use moobot_pgv-msg:lane_detected instead.")
  (lane_detected m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <pgv_scan_data>) ostream)
  "Serializes a message object of type '<pgv_scan_data>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x_pos))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y_pos))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'orientation))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'read_barcode) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'tag_num)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'lane_detected)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <pgv_scan_data>) istream)
  "Deserializes a message object of type '<pgv_scan_data>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x_pos) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y_pos) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'orientation) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'read_barcode) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'tag_num) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'lane_detected) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<pgv_scan_data>)))
  "Returns string type for a message object of type '<pgv_scan_data>"
  "moobot_pgv/pgv_scan_data")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'pgv_scan_data)))
  "Returns string type for a message object of type 'pgv_scan_data"
  "moobot_pgv/pgv_scan_data")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<pgv_scan_data>)))
  "Returns md5sum for a message object of type '<pgv_scan_data>"
  "fb34df9bdee5103f0f83bb286c7a7b50")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'pgv_scan_data)))
  "Returns md5sum for a message object of type 'pgv_scan_data"
  "fb34df9bdee5103f0f83bb286c7a7b50")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<pgv_scan_data>)))
  "Returns full string definition for message of type '<pgv_scan_data>"
  (cl:format cl:nil "float32 x_pos~%float32 y_pos~%float32 orientation~%bool read_barcode~%int32 tag_num~%int32 lane_detected #0 when lane_detected~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'pgv_scan_data)))
  "Returns full string definition for message of type 'pgv_scan_data"
  (cl:format cl:nil "float32 x_pos~%float32 y_pos~%float32 orientation~%bool read_barcode~%int32 tag_num~%int32 lane_detected #0 when lane_detected~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <pgv_scan_data>))
  (cl:+ 0
     4
     4
     4
     1
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <pgv_scan_data>))
  "Converts a ROS message object to a list"
  (cl:list 'pgv_scan_data
    (cl:cons ':x_pos (x_pos msg))
    (cl:cons ':y_pos (y_pos msg))
    (cl:cons ':orientation (orientation msg))
    (cl:cons ':read_barcode (read_barcode msg))
    (cl:cons ':tag_num (tag_num msg))
    (cl:cons ':lane_detected (lane_detected msg))
))
