
(cl:in-package :asdf)

(defsystem "eskf_navi-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "hedge_pos_a" :depends-on ("_package_hedge_pos_a"))
    (:file "_package_hedge_pos_a" :depends-on ("_package"))
  ))