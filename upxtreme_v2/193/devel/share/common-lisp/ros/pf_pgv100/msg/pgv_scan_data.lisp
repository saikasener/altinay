; Auto-generated. Do not edit!


(cl:in-package pf_pgv100-msg)


;//! \htmlinclude pgv_scan_data.msg.html

(cl:defclass <pgv_scan_data> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (angle
    :reader angle
    :initarg :angle
    :type cl:float
    :initform 0.0)
   (x_pos
    :reader x_pos
    :initarg :x_pos
    :type cl:float
    :initform 0.0)
   (y_pos
    :reader y_pos
    :initarg :y_pos
    :type cl:float
    :initform 0.0)
   (direction
    :reader direction
    :initarg :direction
    :type cl:string
    :initform "")
   (color_lane_count
    :reader color_lane_count
    :initarg :color_lane_count
    :type cl:fixnum
    :initform 0)
   (no_color_lane
    :reader no_color_lane
    :initarg :no_color_lane
    :type cl:fixnum
    :initform 0)
   (no_pos
    :reader no_pos
    :initarg :no_pos
    :type cl:fixnum
    :initform 0)
   (tag_detected
    :reader tag_detected
    :initarg :tag_detected
    :type cl:fixnum
    :initform 0)
   (id
    :reader id
    :initarg :id
    :type cl:fixnum
    :initform 0))
)

(cl:defclass pgv_scan_data (<pgv_scan_data>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <pgv_scan_data>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'pgv_scan_data)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pf_pgv100-msg:<pgv_scan_data> is deprecated: use pf_pgv100-msg:pgv_scan_data instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <pgv_scan_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pf_pgv100-msg:header-val is deprecated.  Use pf_pgv100-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <pgv_scan_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pf_pgv100-msg:angle-val is deprecated.  Use pf_pgv100-msg:angle instead.")
  (angle m))

(cl:ensure-generic-function 'x_pos-val :lambda-list '(m))
(cl:defmethod x_pos-val ((m <pgv_scan_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pf_pgv100-msg:x_pos-val is deprecated.  Use pf_pgv100-msg:x_pos instead.")
  (x_pos m))

(cl:ensure-generic-function 'y_pos-val :lambda-list '(m))
(cl:defmethod y_pos-val ((m <pgv_scan_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pf_pgv100-msg:y_pos-val is deprecated.  Use pf_pgv100-msg:y_pos instead.")
  (y_pos m))

(cl:ensure-generic-function 'direction-val :lambda-list '(m))
(cl:defmethod direction-val ((m <pgv_scan_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pf_pgv100-msg:direction-val is deprecated.  Use pf_pgv100-msg:direction instead.")
  (direction m))

(cl:ensure-generic-function 'color_lane_count-val :lambda-list '(m))
(cl:defmethod color_lane_count-val ((m <pgv_scan_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pf_pgv100-msg:color_lane_count-val is deprecated.  Use pf_pgv100-msg:color_lane_count instead.")
  (color_lane_count m))

(cl:ensure-generic-function 'no_color_lane-val :lambda-list '(m))
(cl:defmethod no_color_lane-val ((m <pgv_scan_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pf_pgv100-msg:no_color_lane-val is deprecated.  Use pf_pgv100-msg:no_color_lane instead.")
  (no_color_lane m))

(cl:ensure-generic-function 'no_pos-val :lambda-list '(m))
(cl:defmethod no_pos-val ((m <pgv_scan_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pf_pgv100-msg:no_pos-val is deprecated.  Use pf_pgv100-msg:no_pos instead.")
  (no_pos m))

(cl:ensure-generic-function 'tag_detected-val :lambda-list '(m))
(cl:defmethod tag_detected-val ((m <pgv_scan_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pf_pgv100-msg:tag_detected-val is deprecated.  Use pf_pgv100-msg:tag_detected instead.")
  (tag_detected m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <pgv_scan_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pf_pgv100-msg:id-val is deprecated.  Use pf_pgv100-msg:id instead.")
  (id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <pgv_scan_data>) ostream)
  "Serializes a message object of type '<pgv_scan_data>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
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
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'direction))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'direction))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'color_lane_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'no_color_lane)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'no_pos)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tag_detected)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <pgv_scan_data>) istream)
  "Deserializes a message object of type '<pgv_scan_data>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle) (roslisp-utils:decode-single-float-bits bits)))
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
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'direction) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'direction) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'color_lane_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'no_color_lane)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'no_pos)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tag_detected)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<pgv_scan_data>)))
  "Returns string type for a message object of type '<pgv_scan_data>"
  "pf_pgv100/pgv_scan_data")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'pgv_scan_data)))
  "Returns string type for a message object of type 'pgv_scan_data"
  "pf_pgv100/pgv_scan_data")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<pgv_scan_data>)))
  "Returns md5sum for a message object of type '<pgv_scan_data>"
  "7d3a20e54967aa3cdd07e4fe0a6961b2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'pgv_scan_data)))
  "Returns md5sum for a message object of type 'pgv_scan_data"
  "7d3a20e54967aa3cdd07e4fe0a6961b2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<pgv_scan_data>)))
  "Returns full string definition for message of type '<pgv_scan_data>"
  (cl:format cl:nil "Header header~%float32 angle~%float32 x_pos~%float32 y_pos~%string direction~%uint8 color_lane_count~%uint8 no_color_lane~%uint8 no_pos~%uint8 tag_detected~%uint8 id~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'pgv_scan_data)))
  "Returns full string definition for message of type 'pgv_scan_data"
  (cl:format cl:nil "Header header~%float32 angle~%float32 x_pos~%float32 y_pos~%string direction~%uint8 color_lane_count~%uint8 no_color_lane~%uint8 no_pos~%uint8 tag_detected~%uint8 id~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <pgv_scan_data>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
     4 (cl:length (cl:slot-value msg 'direction))
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <pgv_scan_data>))
  "Converts a ROS message object to a list"
  (cl:list 'pgv_scan_data
    (cl:cons ':header (header msg))
    (cl:cons ':angle (angle msg))
    (cl:cons ':x_pos (x_pos msg))
    (cl:cons ':y_pos (y_pos msg))
    (cl:cons ':direction (direction msg))
    (cl:cons ':color_lane_count (color_lane_count msg))
    (cl:cons ':no_color_lane (no_color_lane msg))
    (cl:cons ':no_pos (no_pos msg))
    (cl:cons ':tag_detected (tag_detected msg))
    (cl:cons ':id (id msg))
))
