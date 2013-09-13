
(cl:in-package :asdf)

(defsystem "hovercraft-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "BatteryVoltage" :depends-on ("_package_BatteryVoltage"))
    (:file "_package_BatteryVoltage" :depends-on ("_package"))
    (:file "Thruster" :depends-on ("_package_Thruster"))
    (:file "_package_Thruster" :depends-on ("_package"))
    (:file "Current" :depends-on ("_package_Current"))
    (:file "_package_Current" :depends-on ("_package"))
    (:file "Gyro" :depends-on ("_package_Gyro"))
    (:file "_package_Gyro" :depends-on ("_package"))
    (:file "LED" :depends-on ("_package_LED"))
    (:file "_package_LED" :depends-on ("_package"))
  ))