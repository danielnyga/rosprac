(asdf:defsystem cram-robot-memory-evaluation
  :depends-on (cram-robot-memory-demo)
  :components
  ((:module "src"
    :components
    ((:file "package")
     (:file "kitchen-evaluator" :depends-on ("package"))))))
