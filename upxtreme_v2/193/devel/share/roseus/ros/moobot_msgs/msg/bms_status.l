;; Auto-generated. Do not edit!


(when (boundp 'moobot_msgs::bms_status)
  (if (not (find-package "MOOBOT_MSGS"))
    (make-package "MOOBOT_MSGS"))
  (shadow 'bms_status (find-package "MOOBOT_MSGS")))
(unless (find-package "MOOBOT_MSGS::BMS_STATUS")
  (make-package "MOOBOT_MSGS::BMS_STATUS"))

(in-package "ROS")
;;//! \htmlinclude bms_status.msg.html


(defclass moobot_msgs::bms_status
  :super ros::object
  :slots (_battery_voltage _load_voltage _charger_voltage _cell_voltage _temperature_bms _temperature_cell _current_income _capacity _capacity_max _soc _current_charger _current_load ))

(defmethod moobot_msgs::bms_status
  (:init
   (&key
    ((:battery_voltage __battery_voltage) 0.0)
    ((:load_voltage __load_voltage) 0.0)
    ((:charger_voltage __charger_voltage) 0.0)
    ((:cell_voltage __cell_voltage) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:temperature_bms __temperature_bms) 0.0)
    ((:temperature_cell __temperature_cell) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:current_income __current_income) 0.0)
    ((:capacity __capacity) 0.0)
    ((:capacity_max __capacity_max) 0.0)
    ((:soc __soc) 0.0)
    ((:current_charger __current_charger) 0.0)
    ((:current_load __current_load) 0.0)
    )
   (send-super :init)
   (setq _battery_voltage (float __battery_voltage))
   (setq _load_voltage (float __load_voltage))
   (setq _charger_voltage (float __charger_voltage))
   (setq _cell_voltage __cell_voltage)
   (setq _temperature_bms (float __temperature_bms))
   (setq _temperature_cell __temperature_cell)
   (setq _current_income (float __current_income))
   (setq _capacity (float __capacity))
   (setq _capacity_max (float __capacity_max))
   (setq _soc (float __soc))
   (setq _current_charger (float __current_charger))
   (setq _current_load (float __current_load))
   self)
  (:battery_voltage
   (&optional __battery_voltage)
   (if __battery_voltage (setq _battery_voltage __battery_voltage)) _battery_voltage)
  (:load_voltage
   (&optional __load_voltage)
   (if __load_voltage (setq _load_voltage __load_voltage)) _load_voltage)
  (:charger_voltage
   (&optional __charger_voltage)
   (if __charger_voltage (setq _charger_voltage __charger_voltage)) _charger_voltage)
  (:cell_voltage
   (&optional __cell_voltage)
   (if __cell_voltage (setq _cell_voltage __cell_voltage)) _cell_voltage)
  (:temperature_bms
   (&optional __temperature_bms)
   (if __temperature_bms (setq _temperature_bms __temperature_bms)) _temperature_bms)
  (:temperature_cell
   (&optional __temperature_cell)
   (if __temperature_cell (setq _temperature_cell __temperature_cell)) _temperature_cell)
  (:current_income
   (&optional __current_income)
   (if __current_income (setq _current_income __current_income)) _current_income)
  (:capacity
   (&optional __capacity)
   (if __capacity (setq _capacity __capacity)) _capacity)
  (:capacity_max
   (&optional __capacity_max)
   (if __capacity_max (setq _capacity_max __capacity_max)) _capacity_max)
  (:soc
   (&optional __soc)
   (if __soc (setq _soc __soc)) _soc)
  (:current_charger
   (&optional __current_charger)
   (if __current_charger (setq _current_charger __current_charger)) _current_charger)
  (:current_load
   (&optional __current_load)
   (if __current_load (setq _current_load __current_load)) _current_load)
  (:serialization-length
   ()
   (+
    ;; float32 _battery_voltage
    4
    ;; float32 _load_voltage
    4
    ;; float32 _charger_voltage
    4
    ;; float32[] _cell_voltage
    (* 4    (length _cell_voltage)) 4
    ;; float32 _temperature_bms
    4
    ;; float32[] _temperature_cell
    (* 4    (length _temperature_cell)) 4
    ;; float32 _current_income
    4
    ;; float32 _capacity
    4
    ;; float32 _capacity_max
    4
    ;; float32 _soc
    4
    ;; float32 _current_charger
    4
    ;; float32 _current_load
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _battery_voltage
       (sys::poke _battery_voltage (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _load_voltage
       (sys::poke _load_voltage (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _charger_voltage
       (sys::poke _charger_voltage (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32[] _cell_voltage
     (write-long (length _cell_voltage) s)
     (dotimes (i (length _cell_voltage))
       (sys::poke (elt _cell_voltage i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32 _temperature_bms
       (sys::poke _temperature_bms (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32[] _temperature_cell
     (write-long (length _temperature_cell) s)
     (dotimes (i (length _temperature_cell))
       (sys::poke (elt _temperature_cell i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32 _current_income
       (sys::poke _current_income (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _capacity
       (sys::poke _capacity (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _capacity_max
       (sys::poke _capacity_max (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _soc
       (sys::poke _soc (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _current_charger
       (sys::poke _current_charger (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _current_load
       (sys::poke _current_load (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _battery_voltage
     (setq _battery_voltage (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _load_voltage
     (setq _load_voltage (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _charger_voltage
     (setq _charger_voltage (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32[] _cell_voltage
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _cell_voltage (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _cell_voltage i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32 _temperature_bms
     (setq _temperature_bms (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32[] _temperature_cell
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _temperature_cell (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _temperature_cell i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32 _current_income
     (setq _current_income (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _capacity
     (setq _capacity (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _capacity_max
     (setq _capacity_max (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _soc
     (setq _soc (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _current_charger
     (setq _current_charger (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _current_load
     (setq _current_load (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get moobot_msgs::bms_status :md5sum-) "08c0d01e45dd995f3adfffdf7aa31d50")
(setf (get moobot_msgs::bms_status :datatype-) "moobot_msgs/bms_status")
(setf (get moobot_msgs::bms_status :definition-)
      "float32 battery_voltage  # Voltage in Volts 
float32 load_voltage  # Voltage in Volts 
float32 charger_voltage  # Voltage in Volts 
float32[] cell_voltage   # An array of individual cell voltages for each cell in the pack

float32 temperature_bms      # Temperature in Degrees Celsius (If unmeasured NaN)
float32[] temperature_cell  # An array of individual cell temperatures for each cell in the pack

float32 current_income     # Pile gelen akım      
float32 capacity           # Pilin doluluk kapasitesi -> Capacity in Ah (last full capacity) 
float32 capacity_max  	    # Pilin total kapasitesi -> Capacity in Ah (design capacity)  
float32 soc                # Charge percentage on 0 to 1 range -> percentage
float32 current_charger    # A     
float32 current_load       # Sistemin Çektiği Akım -> Negative when discharging (A)  

#float32 battery_volt_1
#float32 battery_volt_2
#float32 battery_volt_total
#float32 battery_temp_1
#float32 battery_temp_2
#float32 current_main
#float32 total_power_wh


")



(provide :moobot_msgs/bms_status "08c0d01e45dd995f3adfffdf7aa31d50")


