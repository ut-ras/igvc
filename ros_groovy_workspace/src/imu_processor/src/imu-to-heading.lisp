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

(defmacro defnode (name args &body body)
  (let ((spin (if (eql (car body) :spin) (cadr body) nil))
	(real-body (if (eql (car body) :spin) (cddr body) body)))
    `(defun ,name ()
       (with-ros-node (,(string name) :spin ,spin)
	 ((lambda ,args ,@real-body)
	  ,@(loop for argument in args
	       unless (eq argument '&optional)
	       collect (destructuring-bind (arg-name default) argument
			 `(if (has-param ,(string arg-name))
			      (get-param ,(string arg-name))
			      ,default))))))))

(defnode converter (&optional (filter '(1/2 1/4 1/2)))
    :spin t
    (let ((pub (advertise "/heading" 'std_msgs-msg:Float64))
	  (comp-filter (mk-filter filter)))
      (subscribe "/raw" 'sensor_msgs-msg:Imu
		 (lambda (message)
		   (let ((quat (sensor_msgs-msg:orientation message)))
		     (publish pub
			      (make-instance
			       'std_msgs-msg:Float64
			       :data (funcall comp-filter
					      (yaw-from-quaternion
					       (geometry_msgs-msg:x quat) (geometry_msgs-msg:y quat)
					       (geometry_msgs-msg:z quat) (geometry_msgs-msg:w quat))))))))))

(defnode imu-mock-up (&optional (data-type nil))
  (let ((pub (advertise "/imu/vn200/raw" 'sensor_msgs-msg:Imu))
	(data-generation-func (if (null data-type)
				  (lambda nil (random 10))
				  (lambda nil data-type))))
    (loop-at-most-every 0.1
	 (let ((x (funcall data-generation-func)) (y (funcall data-generation-func))
	       (z (funcall data-generation-func)) (w (funcall data-generation-func)))
	   (let ((denom (sqrt (+ (* x x) (* y y) (* z z) (* w w)))))
	     (publish pub (make-instance 'sensor_msgs-msg:Imu
					 :orientation
					 (if (eql denom 0.0) 
					     (make-instance 'geometry_msgs-msg:quaternion
							    :x x :y y
							    :z z :w w)
					     (make-instance 'geometry_msgs-msg:quaternion
							    :x (/ x denom) :y (/ y denom)
							    :z (/ z denom) :w (/ w denom))))))))))
