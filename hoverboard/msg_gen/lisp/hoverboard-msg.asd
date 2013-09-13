
(cl:in-package :asdf)

(defsystem "hoverboard-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ServoRaw" :depends-on ("_package_ServoRaw"))
    (:file "_package_ServoRaw" :depends-on ("_package"))
    (:file "GPIORaw" :depends-on ("_package_GPIORaw"))
    (:file "_package_GPIORaw" :depends-on ("_package"))
    (:file "ADCRaw" :depends-on ("_package_ADCRaw"))
    (:file "_package_ADCRaw" :depends-on ("_package"))
    (:file "PWMRaw" :depends-on ("_package_PWMRaw"))
    (:file "_package_PWMRaw" :depends-on ("_package"))
    (:file "GyroRaw" :depends-on ("_package_GyroRaw"))
    (:file "_package_GyroRaw" :depends-on ("_package"))
    (:file "ThrusterRaw" :depends-on ("_package_ThrusterRaw"))
    (:file "_package_ThrusterRaw" :depends-on ("_package"))
    (:file "LEDRaw" :depends-on ("_package_LEDRaw"))
    (:file "_package_LEDRaw" :depends-on ("_package"))
  ))