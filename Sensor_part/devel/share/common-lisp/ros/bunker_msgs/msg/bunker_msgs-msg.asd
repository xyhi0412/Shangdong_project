
(cl:in-package :asdf)

(defsystem "bunker_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "BunkerBmsStatus" :depends-on ("_package_BunkerBmsStatus"))
    (:file "_package_BunkerBmsStatus" :depends-on ("_package"))
    (:file "BunkerMotorState" :depends-on ("_package_BunkerMotorState"))
    (:file "_package_BunkerMotorState" :depends-on ("_package"))
    (:file "BunkerRsStatus" :depends-on ("_package_BunkerRsStatus"))
    (:file "_package_BunkerRsStatus" :depends-on ("_package"))
    (:file "BunkerStatus" :depends-on ("_package_BunkerStatus"))
    (:file "_package_BunkerStatus" :depends-on ("_package"))
  ))