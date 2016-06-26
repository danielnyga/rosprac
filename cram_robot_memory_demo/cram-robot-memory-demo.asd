(asdf:defsystem cram-robot-memory-demo
  :depends-on (cram-robot-memory-test-utils)
  :components
  ((:module "src"
    :components
    ((:file "package")
     (:file "perceive-and-grasp-mug-demo" :depends-on ("package"))
     (:file "success-estimation-demo" :depends-on ("package"))))))
