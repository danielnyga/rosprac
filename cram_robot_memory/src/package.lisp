(in-package :cl-user)

(defpackage cram-robot-memory
  (:nicknames :crm)
  (:use #:common-lisp #:roslisp)
  (:export with-completed-designator
           learning-failed
           start-learning
           complete-learning))
