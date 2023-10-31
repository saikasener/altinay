;; Auto-generated. Do not edit!


(when (boundp 'moobot_msgs::moobot_scanner)
  (if (not (find-package "MOOBOT_MSGS"))
    (make-package "MOOBOT_MSGS"))
  (shadow 'moobot_scanner (find-package "MOOBOT_MSGS")))
(unless (find-package "MOOBOT_MSGS::MOOBOT_SCANNER")
  (make-package "MOOBOT_MSGS::MOOBOT_SCANNER"))

(in-package "ROS")
;;//! \htmlinclude moobot_scanner.msg.html


(defclass moobot_msgs::moobot_scanner
  :super ros::object
  :slots (_fs_ossd _fs_w1 _fs_w2 _bs_ossd _bs_w1 _bs_w2 ))

(defmethod moobot_msgs::moobot_scanner
  (:init
   (&key
    ((:fs_ossd __fs_ossd) 0)
    ((:fs_w1 __fs_w1) 0)
    ((:fs_w2 __fs_w2) 0)
    ((:bs_ossd __bs_ossd) 0)
    ((:bs_w1 __bs_w1) 0)
    ((:bs_w2 __bs_w2) 0)
    )
   (send-super :init)
   (setq _fs_ossd (round __fs_ossd))
   (setq _fs_w1 (round __fs_w1))
   (setq _fs_w2 (round __fs_w2))
   (setq _bs_ossd (round __bs_ossd))
   (setq _bs_w1 (round __bs_w1))
   (setq _bs_w2 (round __bs_w2))
   self)
  (:fs_ossd
   (&optional __fs_ossd)
   (if __fs_ossd (setq _fs_ossd __fs_ossd)) _fs_ossd)
  (:fs_w1
   (&optional __fs_w1)
   (if __fs_w1 (setq _fs_w1 __fs_w1)) _fs_w1)
  (:fs_w2
   (&optional __fs_w2)
   (if __fs_w2 (setq _fs_w2 __fs_w2)) _fs_w2)
  (:bs_ossd
   (&optional __bs_ossd)
   (if __bs_ossd (setq _bs_ossd __bs_ossd)) _bs_ossd)
  (:bs_w1
   (&optional __bs_w1)
   (if __bs_w1 (setq _bs_w1 __bs_w1)) _bs_w1)
  (:bs_w2
   (&optional __bs_w2)
   (if __bs_w2 (setq _bs_w2 __bs_w2)) _bs_w2)
  (:serialization-length
   ()
   (+
    ;; byte _fs_ossd
    1
    ;; byte _fs_w1
    1
    ;; byte _fs_w2
    1
    ;; byte _bs_ossd
    1
    ;; byte _bs_w1
    1
    ;; byte _bs_w2
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; byte _fs_ossd
       (write-byte _fs_ossd s)
     ;; byte _fs_w1
       (write-byte _fs_w1 s)
     ;; byte _fs_w2
       (write-byte _fs_w2 s)
     ;; byte _bs_ossd
       (write-byte _bs_ossd s)
     ;; byte _bs_w1
       (write-byte _bs_w1 s)
     ;; byte _bs_w2
       (write-byte _bs_w2 s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; byte _fs_ossd
     (setq _fs_ossd (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _fs_ossd 127) (setq _fs_ossd (- _fs_ossd 256)))
   ;; byte _fs_w1
     (setq _fs_w1 (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _fs_w1 127) (setq _fs_w1 (- _fs_w1 256)))
   ;; byte _fs_w2
     (setq _fs_w2 (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _fs_w2 127) (setq _fs_w2 (- _fs_w2 256)))
   ;; byte _bs_ossd
     (setq _bs_ossd (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _bs_ossd 127) (setq _bs_ossd (- _bs_ossd 256)))
   ;; byte _bs_w1
     (setq _bs_w1 (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _bs_w1 127) (setq _bs_w1 (- _bs_w1 256)))
   ;; byte _bs_w2
     (setq _bs_w2 (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _bs_w2 127) (setq _bs_w2 (- _bs_w2 256)))
   ;;
   self)
  )

(setf (get moobot_msgs::moobot_scanner :md5sum-) "b971113dac55cb95f6b07a91c6331c49")
(setf (get moobot_msgs::moobot_scanner :datatype-) "moobot_msgs/moobot_scanner")
(setf (get moobot_msgs::moobot_scanner :definition-)
      "byte fs_ossd
byte fs_w1
byte fs_w2
byte bs_ossd
byte bs_w1
byte bs_w2
")



(provide :moobot_msgs/moobot_scanner "b971113dac55cb95f6b07a91c6331c49")


