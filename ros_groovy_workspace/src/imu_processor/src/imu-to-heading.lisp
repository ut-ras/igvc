(defpackage :imu-to-heading
  (:use :cl :asdf :roslisp)
  (:export :converter))
(in-package :imu-to-heading)

(defun converter ()
  (with-ros-node ("heading")
    (let ((pub (advertise "heading" 'std_msgs-msg:Float64)))
      (loop-at-most-every .1
	 (publish pub (make-instance 'std_msgs-msg:Float64 :data 1.0))))))
				     
