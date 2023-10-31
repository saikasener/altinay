; Auto-generated. Do not edit!


(cl:in-package moobot_msgs-msg)


;//! \htmlinclude bms_status.msg.html

(cl:defclass <bms_status> (roslisp-msg-protocol:ros-message)
  ((battery_voltage
    :reader battery_voltage
    :initarg :battery_voltage
    :type cl:float
    :initform 0.0)
   (load_voltage
    :reader load_voltage
    :initarg :load_voltage
    :type cl:float
    :initform 0.0)
   (charger_voltage
    :reader charger_voltage
    :initarg :charger_voltage
    :type cl:float
    :initform 0.0)
   (cell_voltage
    :reader cell_voltage
    :initarg :cell_voltage
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (temperature_bms
    :reader temperature_bms
    :initarg :temperature_bms
    :type cl:float
    :initform 0.0)
   (temperature_cell
    :reader temperature_cell
    :initarg :temperature_cell
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (current_income
    :reader current_income
    :initarg :current_income
    :type cl:float
    :initform 0.0)
   (capacity
    :reader capacity
    :initarg :capacity
    :type cl:float
    :initform 0.0)
   (capacity_max
    :reader capacity_max
    :initarg :capacity_max
    :type cl:float
    :initform 0.0)
   (soc
    :reader soc
    :initarg :soc
    :type cl:float
    :initform 0.0)
   (current_charger
    :reader current_charger
    :initarg :current_charger
    :type cl:float
    :initform 0.0)
   (current_load
    :reader current_load
    :initarg :current_load
    :type cl:float
    :initform 0.0))
)

(cl:defclass bms_status (<bms_status>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <bms_status>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'bms_status)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name moobot_msgs-msg:<bms_status> is deprecated: use moobot_msgs-msg:bms_status instead.")))

(cl:ensure-generic-function 'battery_voltage-val :lambda-list '(m))
(cl:defmethod battery_voltage-val ((m <bms_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_msgs-msg:battery_voltage-val is deprecated.  Use moobot_msgs-msg:battery_voltage instead.")
  (battery_voltage m))

(cl:ensure-generic-function 'load_voltage-val :lambda-list '(m))
(cl:defmethod load_voltage-val ((m <bms_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_msgs-msg:load_voltage-val is deprecated.  Use moobot_msgs-msg:load_voltage instead.")
  (load_voltage m))

(cl:ensure-generic-function 'charger_voltage-val :lambda-list '(m))
(cl:defmethod charger_voltage-val ((m <bms_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_msgs-msg:charger_voltage-val is deprecated.  Use moobot_msgs-msg:charger_voltage instead.")
  (charger_voltage m))

(cl:ensure-generic-function 'cell_voltage-val :lambda-list '(m))
(cl:defmethod cell_voltage-val ((m <bms_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_msgs-msg:cell_voltage-val is deprecated.  Use moobot_msgs-msg:cell_voltage instead.")
  (cell_voltage m))

(cl:ensure-generic-function 'temperature_bms-val :lambda-list '(m))
(cl:defmethod temperature_bms-val ((m <bms_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_msgs-msg:temperature_bms-val is deprecated.  Use moobot_msgs-msg:temperature_bms instead.")
  (temperature_bms m))

(cl:ensure-generic-function 'temperature_cell-val :lambda-list '(m))
(cl:defmethod temperature_cell-val ((m <bms_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_msgs-msg:temperature_cell-val is deprecated.  Use moobot_msgs-msg:temperature_cell instead.")
  (temperature_cell m))

(cl:ensure-generic-function 'current_income-val :lambda-list '(m))
(cl:defmethod current_income-val ((m <bms_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_msgs-msg:current_income-val is deprecated.  Use moobot_msgs-msg:current_income instead.")
  (current_income m))

(cl:ensure-generic-function 'capacity-val :lambda-list '(m))
(cl:defmethod capacity-val ((m <bms_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_msgs-msg:capacity-val is deprecated.  Use moobot_msgs-msg:capacity instead.")
  (capacity m))

(cl:ensure-generic-function 'capacity_max-val :lambda-list '(m))
(cl:defmethod capacity_max-val ((m <bms_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_msgs-msg:capacity_max-val is deprecated.  Use moobot_msgs-msg:capacity_max instead.")
  (capacity_max m))

(cl:ensure-generic-function 'soc-val :lambda-list '(m))
(cl:defmethod soc-val ((m <bms_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_msgs-msg:soc-val is deprecated.  Use moobot_msgs-msg:soc instead.")
  (soc m))

(cl:ensure-generic-function 'current_charger-val :lambda-list '(m))
(cl:defmethod current_charger-val ((m <bms_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_msgs-msg:current_charger-val is deprecated.  Use moobot_msgs-msg:current_charger instead.")
  (current_charger m))

(cl:ensure-generic-function 'current_load-val :lambda-list '(m))
(cl:defmethod current_load-val ((m <bms_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moobot_msgs-msg:current_load-val is deprecated.  Use moobot_msgs-msg:current_load instead.")
  (current_load m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <bms_status>) ostream)
  "Serializes a message object of type '<bms_status>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'battery_voltage))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'load_voltage))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'charger_voltage))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'cell_voltage))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'cell_voltage))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'temperature_bms))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'temperature_cell))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'temperature_cell))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'current_income))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'capacity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'capacity_max))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'soc))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'current_charger))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'current_load))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <bms_status>) istream)
  "Deserializes a message object of type '<bms_status>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'battery_voltage) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'load_voltage) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'charger_voltage) (roslisp-utils:decode-single-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'cell_voltage) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'cell_voltage)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'temperature_bms) (roslisp-utils:decode-single-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'temperature_cell) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'temperature_cell)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current_income) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'capacity) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'capacity_max) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'soc) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current_charger) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current_load) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<bms_status>)))
  "Returns string type for a message object of type '<bms_status>"
  "moobot_msgs/bms_status")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'bms_status)))
  "Returns string type for a message object of type 'bms_status"
  "moobot_msgs/bms_status")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<bms_status>)))
  "Returns md5sum for a message object of type '<bms_status>"
  "08c0d01e45dd995f3adfffdf7aa31d50")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'bms_status)))
  "Returns md5sum for a message object of type 'bms_status"
  "08c0d01e45dd995f3adfffdf7aa31d50")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<bms_status>)))
  "Returns full string definition for message of type '<bms_status>"
  (cl:format cl:nil "float32 battery_voltage  # Voltage in Volts ~%float32 load_voltage  # Voltage in Volts ~%float32 charger_voltage  # Voltage in Volts ~%float32[] cell_voltage   # An array of individual cell voltages for each cell in the pack~%~%float32 temperature_bms      # Temperature in Degrees Celsius (If unmeasured NaN)~%float32[] temperature_cell  # An array of individual cell temperatures for each cell in the pack~%~%float32 current_income     # Pile gelen akım      ~%float32 capacity           # Pilin doluluk kapasitesi -> Capacity in Ah (last full capacity) ~%float32 capacity_max  	    # Pilin total kapasitesi -> Capacity in Ah (design capacity)  ~%float32 soc                # Charge percentage on 0 to 1 range -> percentage~%float32 current_charger    # A     ~%float32 current_load       # Sistemin Çektiği Akım -> Negative when discharging (A)  ~%~%#float32 battery_volt_1~%#float32 battery_volt_2~%#float32 battery_volt_total~%#float32 battery_temp_1~%#float32 battery_temp_2~%#float32 current_main~%#float32 total_power_wh~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'bms_status)))
  "Returns full string definition for message of type 'bms_status"
  (cl:format cl:nil "float32 battery_voltage  # Voltage in Volts ~%float32 load_voltage  # Voltage in Volts ~%float32 charger_voltage  # Voltage in Volts ~%float32[] cell_voltage   # An array of individual cell voltages for each cell in the pack~%~%float32 temperature_bms      # Temperature in Degrees Celsius (If unmeasured NaN)~%float32[] temperature_cell  # An array of individual cell temperatures for each cell in the pack~%~%float32 current_income     # Pile gelen akım      ~%float32 capacity           # Pilin doluluk kapasitesi -> Capacity in Ah (last full capacity) ~%float32 capacity_max  	    # Pilin total kapasitesi -> Capacity in Ah (design capacity)  ~%float32 soc                # Charge percentage on 0 to 1 range -> percentage~%float32 current_charger    # A     ~%float32 current_load       # Sistemin Çektiği Akım -> Negative when discharging (A)  ~%~%#float32 battery_volt_1~%#float32 battery_volt_2~%#float32 battery_volt_total~%#float32 battery_temp_1~%#float32 battery_temp_2~%#float32 current_main~%#float32 total_power_wh~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <bms_status>))
  (cl:+ 0
     4
     4
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'cell_voltage) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'temperature_cell) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <bms_status>))
  "Converts a ROS message object to a list"
  (cl:list 'bms_status
    (cl:cons ':battery_voltage (battery_voltage msg))
    (cl:cons ':load_voltage (load_voltage msg))
    (cl:cons ':charger_voltage (charger_voltage msg))
    (cl:cons ':cell_voltage (cell_voltage msg))
    (cl:cons ':temperature_bms (temperature_bms msg))
    (cl:cons ':temperature_cell (temperature_cell msg))
    (cl:cons ':current_income (current_income msg))
    (cl:cons ':capacity (capacity msg))
    (cl:cons ':capacity_max (capacity_max msg))
    (cl:cons ':soc (soc msg))
    (cl:cons ':current_charger (current_charger msg))
    (cl:cons ':current_load (current_load msg))
))
