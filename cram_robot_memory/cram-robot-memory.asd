(asdf:defsystem cram-robot-memory
  :depends-on (roslisp
               cram-rosmln
               cram-designators
               cram-language
               cl-transforms-stamped
               cl-transforms)
  :components
  ((:module "src"
    :components
    ((:file "package")
     (:file "designator-completion" :depends-on ("package"))))))
