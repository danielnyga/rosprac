(asdf:defsystem cram-rosmln
  :depends-on (roslisp
               cram-language
               rosmln-msg
               rosmln-srv)
  :components
  ((:module "src"
    :components
    ((:file "package")
     (:file "rosmln" :depends-on ("package"))))))
