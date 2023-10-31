;; Auto-generated. Do not edit!


(when (boundp 'pf_pgv100::pgv_dir_msg)
  (if (not (find-package "PF_PGV100"))
    (make-package "PF_PGV100"))
  (shadow 'pgv_dir_msg (find-package "PF_PGV100")))
(unless (find-package "PF_PGV100::PGV_DIR_MSG")
  (make-package "PF_PGV100::PGV_DIR_MSG"))

(in-package "ROS")
;;//! \htmlinclude pgv_dir_msg.msg.html


(defclass pf_pgv100::pgv_dir_msg
  :super ros::object
  :slots (_dir_command ))

(defmethod pf_pgv100::pgv_dir_msg
  (:init
   (&key
    ((:dir_command __dir_command) 0)
    )
   (send-super :init)
   (setq _dir_command (round __dir_command))
   self)
  (:dir_command
   (&optional __dir_command)
   (if __dir_command (setq _dir_command __dir_command)) _dir_command)
  (:serialization-length
   ()
   (+
    ;; uint8 _dir_command
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _dir_command
       (write-byte _dir_command s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _dir_command
     (setq _dir_command (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get pf_pgv100::pgv_dir_msg :md5sum-) "be420e78d04d79ee4d8fd23e20966af0")
(setf (get pf_pgv100::pgv_dir_msg :datatype-) "pf_pgv100/pgv_dir_msg")
(setf (get pf_pgv100::pgv_dir_msg :definition-)
      "uint8 dir_command

")



(provide :pf_pgv100/pgv_dir_msg "be420e78d04d79ee4d8fd23e20966af0")


