(in-package :cl-user)

(defpackage cram-robot-memory-test-utils
  (:nicknames :crmtu)
  (:use #:common-lisp)
  (:export spawn-object
           clean-and-spawn-kitchen-and-robot
           execute-in-environment
           execute-training-and-test-functions))


