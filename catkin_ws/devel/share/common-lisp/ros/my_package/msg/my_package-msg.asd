
(cl:in-package :asdf)

(defsystem "my_package-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Corner" :depends-on ("_package_Corner"))
    (:file "_package_Corner" :depends-on ("_package"))
  ))