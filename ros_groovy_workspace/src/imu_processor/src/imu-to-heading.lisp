(defpackage :imu-to-heading
  (:use :cl :asdf :roslisp)
  (:export :converter))
(in-package :imu-to-heading)

(defun dot-product (lhs rhs)
  (apply #'+ (mapcar #'* lhs rhs)))

(defun mk-filter (freq-response)
  (let ((delayed-data nil)
	(reversed-response (reverse freq-response)))
    (lambda (sample)
      (push sample delayed-data)
      (if (< (length delayed-data) (length freq-response))
	  nil
	  (prog1 (dot-product delayed-data reversed-response)
	    (setf delayed-data (nbutlast delayed-data)))))))

(defun converter ()
  (with-ros-node ("heading")
    (let ((pub (advertise "heading" 'std_msgs-msg:Float64))
	  (filter (mk-filter '(1/2 1/4 1/2))))
      (loop-at-most-every .1
	 (publish pub (make-instance 'std_msgs-msg:Float64 :data (funcall filter 1.0)))))))
				     
