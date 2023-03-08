; Auto-generated. Do not edit!


(cl:in-package Car_Spraying-msg)


;//! \htmlinclude SetTwoAngle.msg.html

(cl:defclass <SetTwoAngle> (roslisp-msg-protocol:ros-message)
  ((id0
    :reader id0
    :initarg :id0
    :type cl:fixnum
    :initform 0)
   (angle0
    :reader angle0
    :initarg :angle0
    :type cl:float
    :initform 0.0)
   (id1
    :reader id1
    :initarg :id1
    :type cl:fixnum
    :initform 0)
   (angle1
    :reader angle1
    :initarg :angle1
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetTwoAngle (<SetTwoAngle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetTwoAngle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetTwoAngle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Car_Spraying-msg:<SetTwoAngle> is deprecated: use Car_Spraying-msg:SetTwoAngle instead.")))

(cl:ensure-generic-function 'id0-val :lambda-list '(m))
(cl:defmethod id0-val ((m <SetTwoAngle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Car_Spraying-msg:id0-val is deprecated.  Use Car_Spraying-msg:id0 instead.")
  (id0 m))

(cl:ensure-generic-function 'angle0-val :lambda-list '(m))
(cl:defmethod angle0-val ((m <SetTwoAngle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Car_Spraying-msg:angle0-val is deprecated.  Use Car_Spraying-msg:angle0 instead.")
  (angle0 m))

(cl:ensure-generic-function 'id1-val :lambda-list '(m))
(cl:defmethod id1-val ((m <SetTwoAngle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Car_Spraying-msg:id1-val is deprecated.  Use Car_Spraying-msg:id1 instead.")
  (id1 m))

(cl:ensure-generic-function 'angle1-val :lambda-list '(m))
(cl:defmethod angle1-val ((m <SetTwoAngle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Car_Spraying-msg:angle1-val is deprecated.  Use Car_Spraying-msg:angle1 instead.")
  (angle1 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetTwoAngle>) ostream)
  "Serializes a message object of type '<SetTwoAngle>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angle0))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id1)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angle1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetTwoAngle>) istream)
  "Deserializes a message object of type '<SetTwoAngle>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id0)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle0) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id1)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle1) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetTwoAngle>)))
  "Returns string type for a message object of type '<SetTwoAngle>"
  "Car_Spraying/SetTwoAngle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetTwoAngle)))
  "Returns string type for a message object of type 'SetTwoAngle"
  "Car_Spraying/SetTwoAngle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetTwoAngle>)))
  "Returns md5sum for a message object of type '<SetTwoAngle>"
  "f1d10f92f012da8da765573aa414a014")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetTwoAngle)))
  "Returns md5sum for a message object of type 'SetTwoAngle"
  "f1d10f92f012da8da765573aa414a014")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetTwoAngle>)))
  "Returns full string definition for message of type '<SetTwoAngle>"
  (cl:format cl:nil "uint8 id0~%float32 angle0~%uint8 id1~%float32 angle1~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetTwoAngle)))
  "Returns full string definition for message of type 'SetTwoAngle"
  (cl:format cl:nil "uint8 id0~%float32 angle0~%uint8 id1~%float32 angle1~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetTwoAngle>))
  (cl:+ 0
     1
     4
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetTwoAngle>))
  "Converts a ROS message object to a list"
  (cl:list 'SetTwoAngle
    (cl:cons ':id0 (id0 msg))
    (cl:cons ':angle0 (angle0 msg))
    (cl:cons ':id1 (id1 msg))
    (cl:cons ':angle1 (angle1 msg))
))
