; Auto-generated. Do not edit!


(cl:in-package assignment_2_2023-msg)


;//! \htmlinclude RobotState.msg.html

(cl:defclass <RobotState> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (vel_x
    :reader vel_x
    :initarg :vel_x
    :type cl:float
    :initform 0.0)
   (vel_z
    :reader vel_z
    :initarg :vel_z
    :type cl:float
    :initform 0.0))
)

(cl:defclass RobotState (<RobotState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RobotState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RobotState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name assignment_2_2023-msg:<RobotState> is deprecated: use assignment_2_2023-msg:RobotState instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <RobotState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader assignment_2_2023-msg:x-val is deprecated.  Use assignment_2_2023-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <RobotState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader assignment_2_2023-msg:y-val is deprecated.  Use assignment_2_2023-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'vel_x-val :lambda-list '(m))
(cl:defmethod vel_x-val ((m <RobotState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader assignment_2_2023-msg:vel_x-val is deprecated.  Use assignment_2_2023-msg:vel_x instead.")
  (vel_x m))

(cl:ensure-generic-function 'vel_z-val :lambda-list '(m))
(cl:defmethod vel_z-val ((m <RobotState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader assignment_2_2023-msg:vel_z-val is deprecated.  Use assignment_2_2023-msg:vel_z instead.")
  (vel_z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RobotState>) ostream)
  "Serializes a message object of type '<RobotState>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'vel_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'vel_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RobotState>) istream)
  "Deserializes a message object of type '<RobotState>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vel_x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vel_z) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RobotState>)))
  "Returns string type for a message object of type '<RobotState>"
  "assignment_2_2023/RobotState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotState)))
  "Returns string type for a message object of type 'RobotState"
  "assignment_2_2023/RobotState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RobotState>)))
  "Returns md5sum for a message object of type '<RobotState>"
  "9a6b8857bb44e9dfbb8aa9b340027ecc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RobotState)))
  "Returns md5sum for a message object of type 'RobotState"
  "9a6b8857bb44e9dfbb8aa9b340027ecc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RobotState>)))
  "Returns full string definition for message of type '<RobotState>"
  (cl:format cl:nil "float64 x~%float64 y~%float64 vel_x~%float64 vel_z~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RobotState)))
  "Returns full string definition for message of type 'RobotState"
  (cl:format cl:nil "float64 x~%float64 y~%float64 vel_x~%float64 vel_z~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RobotState>))
  (cl:+ 0
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RobotState>))
  "Converts a ROS message object to a list"
  (cl:list 'RobotState
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':vel_x (vel_x msg))
    (cl:cons ':vel_z (vel_z msg))
))
