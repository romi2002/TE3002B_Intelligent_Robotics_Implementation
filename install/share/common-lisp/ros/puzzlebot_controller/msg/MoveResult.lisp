; Auto-generated. Do not edit!


(cl:in-package puzzlebot_controller-msg)


;//! \htmlinclude MoveResult.msg.html

(cl:defclass <MoveResult> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass MoveResult (<MoveResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name puzzlebot_controller-msg:<MoveResult> is deprecated: use puzzlebot_controller-msg:MoveResult instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveResult>) ostream)
  "Serializes a message object of type '<MoveResult>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveResult>) istream)
  "Deserializes a message object of type '<MoveResult>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveResult>)))
  "Returns string type for a message object of type '<MoveResult>"
  "puzzlebot_controller/MoveResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveResult)))
  "Returns string type for a message object of type 'MoveResult"
  "puzzlebot_controller/MoveResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveResult>)))
  "Returns md5sum for a message object of type '<MoveResult>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveResult)))
  "Returns md5sum for a message object of type 'MoveResult"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveResult>)))
  "Returns full string definition for message of type '<MoveResult>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveResult)))
  "Returns full string definition for message of type 'MoveResult"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveResult>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveResult>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveResult
))