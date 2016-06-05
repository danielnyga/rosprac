(asdf:defsystem cram-robot-memory
  :depends-on (roslisp
               cram-rosmln)
  :components
  ((:module "src"
    :components
    ((:file "package")
     (:file "designator-completion" :depends-on ("package"))))))
