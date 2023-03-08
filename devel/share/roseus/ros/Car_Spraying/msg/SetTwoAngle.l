;; Auto-generated. Do not edit!


(when (boundp 'Car_Spraying::SetTwoAngle)
  (if (not (find-package "CAR_SPRAYING"))
    (make-package "CAR_SPRAYING"))
  (shadow 'SetTwoAngle (find-package "CAR_SPRAYING")))
(unless (find-package "CAR_SPRAYING::SETTWOANGLE")
  (make-package "CAR_SPRAYING::SETTWOANGLE"))

(in-package "ROS")
;;//! \htmlinclude SetTwoAngle.msg.html


(defclass Car_Spraying::SetTwoAngle
  :super ros::object
  :slots (_id0 _angle0 _id1 _angle1 ))

(defmethod Car_Spraying::SetTwoAngle
  (:init
   (&key
    ((:id0 __id0) 0)
    ((:angle0 __angle0) 0.0)
    ((:id1 __id1) 0)
    ((:angle1 __angle1) 0.0)
    )
   (send-super :init)
   (setq _id0 (round __id0))
   (setq _angle0 (float __angle0))
   (setq _id1 (round __id1))
   (setq _angle1 (float __angle1))
   self)
  (:id0
   (&optional __id0)
   (if __id0 (setq _id0 __id0)) _id0)
  (:angle0
   (&optional __angle0)
   (if __angle0 (setq _angle0 __angle0)) _angle0)
  (:id1
   (&optional __id1)
   (if __id1 (setq _id1 __id1)) _id1)
  (:angle1
   (&optional __angle1)
   (if __angle1 (setq _angle1 __angle1)) _angle1)
  (:serialization-length
   ()
   (+
    ;; uint8 _id0
    1
    ;; float32 _angle0
    4
    ;; uint8 _id1
    1
    ;; float32 _angle1
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _id0
       (write-byte _id0 s)
     ;; float32 _angle0
       (sys::poke _angle0 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint8 _id1
       (write-byte _id1 s)
     ;; float32 _angle1
       (sys::poke _angle1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _id0
     (setq _id0 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _angle0
     (setq _angle0 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint8 _id1
     (setq _id1 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _angle1
     (setq _angle1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get Car_Spraying::SetTwoAngle :md5sum-) "f1d10f92f012da8da765573aa414a014")
(setf (get Car_Spraying::SetTwoAngle :datatype-) "Car_Spraying/SetTwoAngle")
(setf (get Car_Spraying::SetTwoAngle :definition-)
      "uint8 id0
float32 angle0
uint8 id1
float32 angle1
")



(provide :Car_Spraying/SetTwoAngle "f1d10f92f012da8da765573aa414a014")


