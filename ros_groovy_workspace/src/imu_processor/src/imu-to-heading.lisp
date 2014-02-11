(defpackage :imu-to-heading
  (:use :cl :asdf :roslisp)
  (:export :converter
	   :imu-mock-up))
(in-package :imu-to-heading)

(defun mk-filter (freq-response)
  (let ((delayed-data nil)
	(reversed-response (reverse freq-response)))
    (lambda (sample)
      (push sample delayed-data)
      (if (< (length delayed-data) (length freq-response))
	  nil
	  (prog1 (apply #'+ (mapcar #'* delayed-data reversed-response))
	    (setf delayed-data (nbutlast delayed-data)))))))

(defun yaw-from-quaternion (x y z rot)
   (atan (- (* 2 y rot) (* 2 x z)) (- 1 (* 2 y y) (* 2 z z))))

(defun converter ()
  (with-ros-node ("heading" :spin t)
    (let ((pub (advertise "heading" 'std_msgs-msg:Float64))
	  (filter (mk-filter '(1/2 1/4 1/2))))
      (subscribe "/raw" 'sensor_msgs-msg:Imu
		 (lambda (message)
		   (let ((quat (sensor_msgs-msg:orientation message)))
		     (publish pub
		      (make-instance
		       'std_msgs-msg:Float64
		       :data (funcall filter
			      (yaw-from-quaternion
			       (geometry_msgs-msg:x quat) (geometry_msgs-msg:y quat)
			       (geometry_msgs-msg:z quat) (geometry_msgs-msg:w quat)))))))))))

(defun imu-mock-up ()
  (with-ros-node ("imu_mockup")
    (let ((pub (advertise "/imu/vn200/raw" 'sensor_msgs-msg:Imu)))
      (loop-at-most-every 0.1
	   (let ((x (random 10)) (y (random 10))
		 (z (random 10)) (w (random 10)))
	     (let ((denom (sqrt (+ (* x x) (* y y) (* z z) (* w w)))))
	       (publish pub (make-instance 'sensor_msgs-msg:Imu
					   :orientation
					   (make-instance 'geometry_msgs-msg:quaternion
							  :x (/ x denom) :y (/ y denom)
							  :z (/ z denom) :w (/ w denom))))))))))
