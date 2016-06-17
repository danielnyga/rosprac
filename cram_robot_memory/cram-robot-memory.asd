(asdf:defsystem cram-robot-memory
  :depends-on (roslisp
               cram-rosmln
               cram-designators
               cram-language
               cl-transforms-stamped
               owl_memory_converter-srv
               cl-transforms)
  :components
  ((:module "src"
    :components
    ((:file "package")
     (:file "designator-completion" :depends-on ("package"))
     (:file "learning" :depends-on ("package"))))))
