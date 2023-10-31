;; Auto-generated. Do not edit!


(when (boundp 'agv_mode_srvs::agv_mode_srv)
  (if (not (find-package "AGV_MODE_SRVS"))
    (make-package "AGV_MODE_SRVS"))
  (shadow 'agv_mode_srv (find-package "AGV_MODE_SRVS")))
(unless (find-package "AGV_MODE_SRVS::AGV_MODE_SRV")
  (make-package "AGV_MODE_SRVS::AGV_MODE_SRV"))
(unless (find-package "AGV_MODE_SRVS::AGV_MODE_SRVREQUEST")
  (make-package "AGV_MODE_SRVS::AGV_MODE_SRVREQUEST"))
(unless (find-package "AGV_MODE_SRVS::AGV_MODE_SRVRESPONSE")
  (make-package "AGV_MODE_SRVS::AGV_MODE_SRVRESPONSE"))

(in-package "ROS")





(defclass agv_mode_srvs::agv_mode_srvRequest
  :super ros::object
  :slots (_mode ))

(defmethod agv_mode_srvs::agv_mode_srvRequest
  (:init
   (&key
    ((:mode __mode) 0)
    )
   (send-super :init)
   (setq _mode (round __mode))
   self)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:serialization-length
   ()
   (+
    ;; int64 _mode
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _mode
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _mode (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _mode) (= (length (_mode . bv)) 2)) ;; bignum
              (write-long (ash (elt (_mode . bv) 0) 0) s)
              (write-long (ash (elt (_mode . bv) 1) -1) s))
             ((and (class _mode) (= (length (_mode . bv)) 1)) ;; big1
              (write-long (elt (_mode . bv) 0) s)
              (write-long (if (>= _mode 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _mode s)(write-long (if (>= _mode 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _mode
#+(or :alpha :irix6 :x86_64)
      (setf _mode (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _mode (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(defclass agv_mode_srvs::agv_mode_srvResponse
  :super ros::object
  :slots (_newmode ))

(defmethod agv_mode_srvs::agv_mode_srvResponse
  (:init
   (&key
    ((:newmode __newmode) 0)
    )
   (send-super :init)
   (setq _newmode (round __newmode))
   self)
  (:newmode
   (&optional __newmode)
   (if __newmode (setq _newmode __newmode)) _newmode)
  (:serialization-length
   ()
   (+
    ;; int64 _newmode
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _newmode
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _newmode (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _newmode) (= (length (_newmode . bv)) 2)) ;; bignum
              (write-long (ash (elt (_newmode . bv) 0) 0) s)
              (write-long (ash (elt (_newmode . bv) 1) -1) s))
             ((and (class _newmode) (= (length (_newmode . bv)) 1)) ;; big1
              (write-long (elt (_newmode . bv) 0) s)
              (write-long (if (>= _newmode 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _newmode s)(write-long (if (>= _newmode 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _newmode
#+(or :alpha :irix6 :x86_64)
      (setf _newmode (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _newmode (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(defclass agv_mode_srvs::agv_mode_srv
  :super ros::object
  :slots ())

(setf (get agv_mode_srvs::agv_mode_srv :md5sum-) "ae96190e987c19063498c13c3886f347")
(setf (get agv_mode_srvs::agv_mode_srv :datatype-) "agv_mode_srvs/agv_mode_srv")
(setf (get agv_mode_srvs::agv_mode_srv :request) agv_mode_srvs::agv_mode_srvRequest)
(setf (get agv_mode_srvs::agv_mode_srv :response) agv_mode_srvs::agv_mode_srvResponse)

(defmethod agv_mode_srvs::agv_mode_srvRequest
  (:response () (instance agv_mode_srvs::agv_mode_srvResponse :init)))

(setf (get agv_mode_srvs::agv_mode_srvRequest :md5sum-) "ae96190e987c19063498c13c3886f347")
(setf (get agv_mode_srvs::agv_mode_srvRequest :datatype-) "agv_mode_srvs/agv_mode_srvRequest")
(setf (get agv_mode_srvs::agv_mode_srvRequest :definition-)
      "int64 mode
---
int64 newmode

")

(setf (get agv_mode_srvs::agv_mode_srvResponse :md5sum-) "ae96190e987c19063498c13c3886f347")
(setf (get agv_mode_srvs::agv_mode_srvResponse :datatype-) "agv_mode_srvs/agv_mode_srvResponse")
(setf (get agv_mode_srvs::agv_mode_srvResponse :definition-)
      "int64 mode
---
int64 newmode

")



(provide :agv_mode_srvs/agv_mode_srv "ae96190e987c19063498c13c3886f347")


