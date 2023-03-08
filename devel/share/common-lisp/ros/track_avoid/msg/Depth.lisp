; Auto-generated. Do not edit!


(cl:in-package track_avoid-msg)


;//! \htmlinclude Depth.msg.html

(cl:defclass <Depth> (roslisp-msg-protocol:ros-message)
  ((locate
    :reader locate
    :initarg :locate
    :type cl:fixnum
    :initform 0)
   (depth
    :reader depth
    :initarg :depth
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Depth (<Depth>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Depth>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Depth)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name track_avoid-msg:<Depth> is deprecated: use track_avoid-msg:Depth instead.")))

(cl:ensure-generic-function 'locate-val :lambda-list '(m))
(cl:defmethod locate-val ((m <Depth>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader track_avoid-msg:locate-val is deprecated.  Use track_avoid-msg:locate instead.")
  (locate m))

(cl:ensure-generic-function 'depth-val :lambda-list '(m))
(cl:defmethod depth-val ((m <Depth>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader track_avoid-msg:depth-val is deprecated.  Use track_avoid-msg:depth instead.")
  (depth m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Depth>) ostream)
  "Serializes a message object of type '<Depth>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'locate)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'locate)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'depth)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'depth)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Depth>) istream)
  "Deserializes a message object of type '<Depth>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'locate)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'locate)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'depth)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'depth)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Depth>)))
  "Returns string type for a message object of type '<Depth>"
  "track_avoid/Depth")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Depth)))
  "Returns string type for a message object of type 'Depth"
  "track_avoid/Depth")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Depth>)))
  "Returns md5sum for a message object of type '<Depth>"
  "b0af3060455b6b7cf46be4171b872aa5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Depth)))
  "Returns md5sum for a message object of type 'Depth"
  "b0af3060455b6b7cf46be4171b872aa5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Depth>)))
  "Returns full string definition for message of type '<Depth>"
  (cl:format cl:nil "uint16 locate~%uint16 depth~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Depth)))
  "Returns full string definition for message of type 'Depth"
  (cl:format cl:nil "uint16 locate~%uint16 depth~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Depth>))
  (cl:+ 0
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Depth>))
  "Converts a ROS message object to a list"
  (cl:list 'Depth
    (cl:cons ':locate (locate msg))
    (cl:cons ':depth (depth msg))
))
