;; Auto-generated. Do not edit!


(when (boundp 'point_message::statsMsg)
  (if (not (find-package "POINT_MESSAGE"))
    (make-package "POINT_MESSAGE"))
  (shadow 'statsMsg (find-package "POINT_MESSAGE")))
(unless (find-package "POINT_MESSAGE::STATSMSG")
  (make-package "POINT_MESSAGE::STATSMSG"))

(in-package "ROS")
;;//! \htmlinclude statsMsg.msg.html


(defclass point_message::statsMsg
  :super ros::object
  :slots (_xSD _ySD _xCent _yCent ))

(defmethod point_message::statsMsg
  (:init
   (&key
    ((:xSD __xSD) 0)
    ((:ySD __ySD) 0)
    ((:xCent __xCent) 0)
    ((:yCent __yCent) 0)
    )
   (send-super :init)
   (setq _xSD (round __xSD))
   (setq _ySD (round __ySD))
   (setq _xCent (round __xCent))
   (setq _yCent (round __yCent))
   self)
  (:xSD
   (&optional __xSD)
   (if __xSD (setq _xSD __xSD)) _xSD)
  (:ySD
   (&optional __ySD)
   (if __ySD (setq _ySD __ySD)) _ySD)
  (:xCent
   (&optional __xCent)
   (if __xCent (setq _xCent __xCent)) _xCent)
  (:yCent
   (&optional __yCent)
   (if __yCent (setq _yCent __yCent)) _yCent)
  (:serialization-length
   ()
   (+
    ;; int32 _xSD
    4
    ;; int32 _ySD
    4
    ;; int32 _xCent
    4
    ;; int32 _yCent
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _xSD
       (write-long _xSD s)
     ;; int32 _ySD
       (write-long _ySD s)
     ;; int32 _xCent
       (write-long _xCent s)
     ;; int32 _yCent
       (write-long _yCent s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _xSD
     (setq _xSD (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _ySD
     (setq _ySD (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _xCent
     (setq _xCent (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _yCent
     (setq _yCent (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get point_message::statsMsg :md5sum-) "0774c5ae2f41c65f0564a155f4857204")
(setf (get point_message::statsMsg :datatype-) "point_message/statsMsg")
(setf (get point_message::statsMsg :definition-)
      "int32 xSD
int32 ySD
int32 xCent
int32 yCent
")



(provide :point_message/statsMsg "0774c5ae2f41c65f0564a155f4857204")


