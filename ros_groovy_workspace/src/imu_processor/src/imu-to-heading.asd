(require :asdf)

(asdf:defsystem "imu-to-heading"
  :description "A simple processing node that converts from Imu data to a Heading"
  :version "0.0.1"
  :author "Jimmy Brisson <theotherjimmy@gmail.com"
  :licence "BSD"
  :components ((:file "imu-to-heading"))
  :depends-on ("roslisp" "sensor_msgs-msg" "std_msgs-msg"))
