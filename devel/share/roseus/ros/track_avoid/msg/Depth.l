;; Auto-generated. Do not edit!


(when (boundp 'track_avoid::Depth)
  (if (not (find-package "TRACK_AVOID"))
    (make-package "TRACK_AVOID"))
  (shadow 'Depth (find-package "TRACK_AVOID")))
(unless (find-package "TRACK_AVOID::DEPTH")
  (make-package "TRACK_AVOID::DEPTH"))

(in-package "ROS")
;;//! \htmlinclude Depth.msg.html


(defclass track_avoid::Depth
  :super ros::object
  :slots (_locate _depth ))

(defmethod track_avoid::Depth
  (:init
   (&key
    ((:locate __locate) 0)
    ((:depth __depth) 0)
    )
   (send-super :init)
   (setq _locate (round __locate))
   (setq _depth (round __depth))
   self)
  (:locate
   (&optional __locate)
   (if __locate (setq _locate __locate)) _locate)
  (:depth
   (&optional __depth)
   (if __depth (setq _depth __depth)) _depth)
  (:serialization-length
   ()
   (+
    ;; uint16 _locate
    2
    ;; uint16 _depth
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint16 _locate
       (write-word _locate s)
     ;; uint16 _depth
       (write-word _depth s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint16 _locate
     (setq _locate (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _depth
     (setq _depth (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get track_avoid::Depth :md5sum-) "b0af3060455b6b7cf46be4171b872aa5")
(setf (get track_avoid::Depth :datatype-) "track_avoid/Depth")
(setf (get track_avoid::Depth :definition-)
      "uint16 locate
uint16 depth
")



(provide :track_avoid/Depth "b0af3060455b6b7cf46be4171b872aa5")


