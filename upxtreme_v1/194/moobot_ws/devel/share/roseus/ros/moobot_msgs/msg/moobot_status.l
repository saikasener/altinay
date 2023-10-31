;; Auto-generated. Do not edit!


(when (boundp 'moobot_msgs::moobot_status)
  (if (not (find-package "MOOBOT_MSGS"))
    (make-package "MOOBOT_MSGS"))
  (shadow 'moobot_status (find-package "MOOBOT_MSGS")))
(unless (find-package "MOOBOT_MSGS::MOOBOT_STATUS")
  (make-package "MOOBOT_MSGS::MOOBOT_STATUS"))

(in-package "ROS")
;;//! \htmlinclude moobot_status.msg.html


(defclass moobot_msgs::moobot_status
  :super ros::object
  :slots (_agv_mode _agv_stopped _emergency _fault _job ))

(defmethod moobot_msgs::moobot_status
  (:init
   (&key
    ((:agv_mode __agv_mode) 0)
    ((:agv_stopped __agv_stopped) nil)
    ((:emergency __emergency) nil)
    ((:fault __fault) nil)
    ((:job __job) "")
    )
   (send-super :init)
   (setq _agv_mode (round __agv_mode))
   (setq _agv_stopped __agv_stopped)
   (setq _emergency __emergency)
   (setq _fault __fault)
   (setq _job (string __job))
   self)
  (:agv_mode
   (&optional __agv_mode)
   (if __agv_mode (setq _agv_mode __agv_mode)) _agv_mode)
  (:agv_stopped
   (&optional (__agv_stopped :null))
   (if (not (eq __agv_stopped :null)) (setq _agv_stopped __agv_stopped)) _agv_stopped)
  (:emergency
   (&optional (__emergency :null))
   (if (not (eq __emergency :null)) (setq _emergency __emergency)) _emergency)
  (:fault
   (&optional (__fault :null))
   (if (not (eq __fault :null)) (setq _fault __fault)) _fault)
  (:job
   (&optional __job)
   (if __job (setq _job __job)) _job)
  (:serialization-length
   ()
   (+
    ;; int32 _agv_mode
    4
    ;; bool _agv_stopped
    1
    ;; bool _emergency
    1
    ;; bool _fault
    1
    ;; string _job
    4 (length _job)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _agv_mode
       (write-long _agv_mode s)
     ;; bool _agv_stopped
       (if _agv_stopped (write-byte -1 s) (write-byte 0 s))
     ;; bool _emergency
       (if _emergency (write-byte -1 s) (write-byte 0 s))
     ;; bool _fault
       (if _fault (write-byte -1 s) (write-byte 0 s))
     ;; string _job
       (write-long (length _job) s) (princ _job s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _agv_mode
     (setq _agv_mode (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _agv_stopped
     (setq _agv_stopped (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _emergency
     (setq _emergency (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _fault
     (setq _fault (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _job
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _job (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get moobot_msgs::moobot_status :md5sum-) "6ac369f66a5b4dcf60abbe45c6d5c34f")
(setf (get moobot_msgs::moobot_status :datatype-) "moobot_msgs/moobot_status")
(setf (get moobot_msgs::moobot_status :definition-)
      "int32 agv_mode
bool agv_stopped
bool emergency
bool fault
string job
")



(provide :moobot_msgs/moobot_status "6ac369f66a5b4dcf60abbe45c6d5c34f")


