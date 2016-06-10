(in-package :cl-user)

(defpackage cram-robot-memory
  (:nicknames :crm)
  (:use #:common-lisp #:roslisp)
  (:export with-completed-designator invalid-designator-failure))
