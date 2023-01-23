;; Auto-generated. Do not edit!


(when (boundp 'my_package::Corner)
  (if (not (find-package "MY_PACKAGE"))
    (make-package "MY_PACKAGE"))
  (shadow 'Corner (find-package "MY_PACKAGE")))
(unless (find-package "MY_PACKAGE::CORNER")
  (make-package "MY_PACKAGE::CORNER"))

(in-package "ROS")
;;//! \htmlinclude Corner.msg.html


(defclass my_package::Corner
  :super ros::object
  :slots (_corner_x _corner_y _corner_z ))

(defmethod my_package::Corner
  (:init
   (&key
    ((:corner_x __corner_x) 0.0)
    ((:corner_y __corner_y) 0.0)
    ((:corner_z __corner_z) 0.0)
    )
   (send-super :init)
   (setq _corner_x (float __corner_x))
   (setq _corner_y (float __corner_y))
   (setq _corner_z (float __corner_z))
   self)
  (:corner_x
   (&optional __corner_x)
   (if __corner_x (setq _corner_x __corner_x)) _corner_x)
  (:corner_y
   (&optional __corner_y)
   (if __corner_y (setq _corner_y __corner_y)) _corner_y)
  (:corner_z
   (&optional __corner_z)
   (if __corner_z (setq _corner_z __corner_z)) _corner_z)
  (:serialization-length
   ()
   (+
    ;; float32 _corner_x
    4
    ;; float32 _corner_y
    4
    ;; float32 _corner_z
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _corner_x
       (sys::poke _corner_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _corner_y
       (sys::poke _corner_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _corner_z
       (sys::poke _corner_z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _corner_x
     (setq _corner_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _corner_y
     (setq _corner_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _corner_z
     (setq _corner_z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get my_package::Corner :md5sum-) "92060748d9174f56becd0c34fba143e6")
(setf (get my_package::Corner :datatype-) "my_package/Corner")
(setf (get my_package::Corner :definition-)
      "float32 corner_x
float32 corner_y
float32 corner_z

")



(provide :my_package/Corner "92060748d9174f56becd0c34fba143e6")


