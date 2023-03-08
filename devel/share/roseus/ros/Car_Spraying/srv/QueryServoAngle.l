;; Auto-generated. Do not edit!


(when (boundp 'Car_Spraying::QueryServoAngle)
  (if (not (find-package "CAR_SPRAYING"))
    (make-package "CAR_SPRAYING"))
  (shadow 'QueryServoAngle (find-package "CAR_SPRAYING")))
(unless (find-package "CAR_SPRAYING::QUERYSERVOANGLE")
  (make-package "CAR_SPRAYING::QUERYSERVOANGLE"))
(unless (find-package "CAR_SPRAYING::QUERYSERVOANGLEREQUEST")
  (make-package "CAR_SPRAYING::QUERYSERVOANGLEREQUEST"))
(unless (find-package "CAR_SPRAYING::QUERYSERVOANGLERESPONSE")
  (make-package "CAR_SPRAYING::QUERYSERVOANGLERESPONSE"))

(in-package "ROS")





(defclass Car_Spraying::QueryServoAngleRequest
  :super ros::object
  :slots (_id ))

(defmethod Car_Spraying::QueryServoAngleRequest
  (:init
   (&key
    ((:id __id) 0)
    )
   (send-super :init)
   (setq _id (round __id))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:serialization-length
   ()
   (+
    ;; uint8 _id
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _id
       (write-byte _id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _id
     (setq _id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(defclass Car_Spraying::QueryServoAngleResponse
  :super ros::object
  :slots (_angle ))

(defmethod Car_Spraying::QueryServoAngleResponse
  (:init
   (&key
    ((:angle __angle) 0.0)
    )
   (send-super :init)
   (setq _angle (float __angle))
   self)
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:serialization-length
   ()
   (+
    ;; float32 _angle
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _angle
       (sys::poke _angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _angle
     (setq _angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass Car_Spraying::QueryServoAngle
  :super ros::object
  :slots ())

(setf (get Car_Spraying::QueryServoAngle :md5sum-) "3fa0610a2c670f4fbf342f6899c09e8c")
(setf (get Car_Spraying::QueryServoAngle :datatype-) "Car_Spraying/QueryServoAngle")
(setf (get Car_Spraying::QueryServoAngle :request) Car_Spraying::QueryServoAngleRequest)
(setf (get Car_Spraying::QueryServoAngle :response) Car_Spraying::QueryServoAngleResponse)

(defmethod Car_Spraying::QueryServoAngleRequest
  (:response () (instance Car_Spraying::QueryServoAngleResponse :init)))

(setf (get Car_Spraying::QueryServoAngleRequest :md5sum-) "3fa0610a2c670f4fbf342f6899c09e8c")
(setf (get Car_Spraying::QueryServoAngleRequest :datatype-) "Car_Spraying/QueryServoAngleRequest")
(setf (get Car_Spraying::QueryServoAngleRequest :definition-)
      "uint8 id
---
float32 angle
")

(setf (get Car_Spraying::QueryServoAngleResponse :md5sum-) "3fa0610a2c670f4fbf342f6899c09e8c")
(setf (get Car_Spraying::QueryServoAngleResponse :datatype-) "Car_Spraying/QueryServoAngleResponse")
(setf (get Car_Spraying::QueryServoAngleResponse :definition-)
      "uint8 id
---
float32 angle
")



(provide :Car_Spraying/QueryServoAngle "3fa0610a2c670f4fbf342f6899c09e8c")


