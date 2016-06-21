(in-package :cl-user)

(defpackage cram-robot-memory
  (:nicknames :crm)
  (:use #:common-lisp #:roslisp)
  (:export with-completed-designator
           complete-with-next-on-failure
           get-failure-probability
           learning-failed
           start-learning
           complete-learning))
