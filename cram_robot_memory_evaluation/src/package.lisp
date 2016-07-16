(in-package :cl-user)

(defpackage cram-robot-memory-evaluation
  (:nicknames :crme)
  (:use #:common-lisp)
  (:export execute-training
           execute-mln-completion-test
           execute-naive-completion-test
           execute-informed-test
           execute-theoretical-test))
