; Auto-generated. Do not edit!


(cl:in-package my_package-msg)


;//! \htmlinclude Corner.msg.html

(cl:defclass <Corner> (roslisp-msg-protocol:ros-message)
  ((corner_x
    :reader corner_x
    :initarg :corner_x
    :type cl:float
    :initform 0.0)
   (corner_y
    :reader corner_y
    :initarg :corner_y
    :type cl:float
    :initform 0.0)
   (corner_z
    :reader corner_z
    :initarg :corner_z
    :type cl:float
    :initform 0.0))
)

(cl:defclass Corner (<Corner>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Corner>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Corner)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name my_package-msg:<Corner> is deprecated: use my_package-msg:Corner instead.")))

(cl:ensure-generic-function 'corner_x-val :lambda-list '(m))
(cl:defmethod corner_x-val ((m <Corner>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_package-msg:corner_x-val is deprecated.  Use my_package-msg:corner_x instead.")
  (corner_x m))

(cl:ensure-generic-function 'corner_y-val :lambda-list '(m))
(cl:defmethod corner_y-val ((m <Corner>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_package-msg:corner_y-val is deprecated.  Use my_package-msg:corner_y instead.")
  (corner_y m))

(cl:ensure-generic-function 'corner_z-val :lambda-list '(m))
(cl:defmethod corner_z-val ((m <Corner>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_package-msg:corner_z-val is deprecated.  Use my_package-msg:corner_z instead.")
  (corner_z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Corner>) ostream)
  "Serializes a message object of type '<Corner>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'corner_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'corner_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'corner_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Corner>) istream)
  "Deserializes a message object of type '<Corner>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'corner_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'corner_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'corner_z) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Corner>)))
  "Returns string type for a message object of type '<Corner>"
  "my_package/Corner")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Corner)))
  "Returns string type for a message object of type 'Corner"
  "my_package/Corner")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Corner>)))
  "Returns md5sum for a message object of type '<Corner>"
  "92060748d9174f56becd0c34fba143e6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Corner)))
  "Returns md5sum for a message object of type 'Corner"
  "92060748d9174f56becd0c34fba143e6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Corner>)))
  "Returns full string definition for message of type '<Corner>"
  (cl:format cl:nil "float32 corner_x~%float32 corner_y~%float32 corner_z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Corner)))
  "Returns full string definition for message of type 'Corner"
  (cl:format cl:nil "float32 corner_x~%float32 corner_y~%float32 corner_z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Corner>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Corner>))
  "Converts a ROS message object to a list"
  (cl:list 'Corner
    (cl:cons ':corner_x (corner_x msg))
    (cl:cons ':corner_y (corner_y msg))
    (cl:cons ':corner_z (corner_z msg))
))
