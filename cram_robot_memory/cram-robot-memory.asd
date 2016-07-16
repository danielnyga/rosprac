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
     (:file "utilities" :depends-on ("package"))
     (:file "general-queries" :depends-on ("package" "utilities"))
     (:file "success-estimation" :depends-on ("package" "utilities" "general-queries"))
     (:file "designator-completion" :depends-on ("package" "utilities"))
     (:file "learning" :depends-on ("package"))))))
