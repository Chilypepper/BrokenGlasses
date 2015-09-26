;; Auto-generated. Do not edit!


(when (boundp 'point_message::pointMsg)
  (if (not (find-package "POINT_MESSAGE"))
    (make-package "POINT_MESSAGE"))
  (shadow 'pointMsg (find-package "POINT_MESSAGE")))
(unless (find-package "POINT_MESSAGE::POINTMSG")
  (make-package "POINT_MESSAGE::POINTMSG"))

(in-package "ROS")
;;//! \htmlinclude pointMsg.msg.html


(defclass point_message::pointMsg
  :super ros::object
  :slots (_xCoor _yCoor ))

(defmethod point_message::pointMsg
  (:init
   (&key
    ((:xCoor __xCoor) 0)
    ((:yCoor __yCoor) 0)
    )
   (send-super :init)
   (setq _xCoor (round __xCoor))
   (setq _yCoor (round __yCoor))
   self)
  (:xCoor
   (&optional __xCoor)
   (if __xCoor (setq _xCoor __xCoor)) _xCoor)
  (:yCoor
   (&optional __yCoor)
   (if __yCoor (setq _yCoor __yCoor)) _yCoor)
  (:serialization-length
   ()
   (+
    ;; int32 _xCoor
    4
    ;; int32 _yCoor
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _xCoor
       (write-long _xCoor s)
     ;; int32 _yCoor
       (write-long _yCoor s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _xCoor
     (setq _xCoor (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _yCoor
     (setq _yCoor (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get point_message::pointMsg :md5sum-) "1a6880393b764025d762728ed326e7f7")
(setf (get point_message::pointMsg :datatype-) "point_message/pointMsg")
(setf (get point_message::pointMsg :definition-)
      "int32 xCoor
int32 yCoor
")



(provide :point_message/pointMsg "1a6880393b764025d762728ed326e7f7")


