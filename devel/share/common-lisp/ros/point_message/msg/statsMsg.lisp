; Auto-generated. Do not edit!


(cl:in-package point_message-msg)


;//! \htmlinclude statsMsg.msg.html

(cl:defclass <statsMsg> (roslisp-msg-protocol:ros-message)
  ((xSD
    :reader xSD
    :initarg :xSD
    :type cl:integer
    :initform 0)
   (ySD
    :reader ySD
    :initarg :ySD
    :type cl:integer
    :initform 0)
   (xCent
    :reader xCent
    :initarg :xCent
    :type cl:integer
    :initform 0)
   (yCent
    :reader yCent
    :initarg :yCent
    :type cl:integer
    :initform 0))
)

(cl:defclass statsMsg (<statsMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <statsMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'statsMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name point_message-msg:<statsMsg> is deprecated: use point_message-msg:statsMsg instead.")))

(cl:ensure-generic-function 'xSD-val :lambda-list '(m))
(cl:defmethod xSD-val ((m <statsMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader point_message-msg:xSD-val is deprecated.  Use point_message-msg:xSD instead.")
  (xSD m))

(cl:ensure-generic-function 'ySD-val :lambda-list '(m))
(cl:defmethod ySD-val ((m <statsMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader point_message-msg:ySD-val is deprecated.  Use point_message-msg:ySD instead.")
  (ySD m))

(cl:ensure-generic-function 'xCent-val :lambda-list '(m))
(cl:defmethod xCent-val ((m <statsMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader point_message-msg:xCent-val is deprecated.  Use point_message-msg:xCent instead.")
  (xCent m))

(cl:ensure-generic-function 'yCent-val :lambda-list '(m))
(cl:defmethod yCent-val ((m <statsMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader point_message-msg:yCent-val is deprecated.  Use point_message-msg:yCent instead.")
  (yCent m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <statsMsg>) ostream)
  "Serializes a message object of type '<statsMsg>"
  (cl:let* ((signed (cl:slot-value msg 'xSD)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'ySD)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'xCent)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'yCent)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <statsMsg>) istream)
  "Deserializes a message object of type '<statsMsg>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'xSD) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ySD) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'xCent) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'yCent) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<statsMsg>)))
  "Returns string type for a message object of type '<statsMsg>"
  "point_message/statsMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'statsMsg)))
  "Returns string type for a message object of type 'statsMsg"
  "point_message/statsMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<statsMsg>)))
  "Returns md5sum for a message object of type '<statsMsg>"
  "0774c5ae2f41c65f0564a155f4857204")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'statsMsg)))
  "Returns md5sum for a message object of type 'statsMsg"
  "0774c5ae2f41c65f0564a155f4857204")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<statsMsg>)))
  "Returns full string definition for message of type '<statsMsg>"
  (cl:format cl:nil "int32 xSD~%int32 ySD~%int32 xCent~%int32 yCent~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'statsMsg)))
  "Returns full string definition for message of type 'statsMsg"
  (cl:format cl:nil "int32 xSD~%int32 ySD~%int32 xCent~%int32 yCent~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <statsMsg>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <statsMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'statsMsg
    (cl:cons ':xSD (xSD msg))
    (cl:cons ':ySD (ySD msg))
    (cl:cons ':xCent (xCent msg))
    (cl:cons ':yCent (yCent msg))
))
