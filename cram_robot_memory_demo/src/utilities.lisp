(in-package :cram-robot-memory-demo)

(defun execute-training-and-test-functions(environment training test)
  (roslisp-utilities::startup-ros)
  (cram-robot-memory:start-learning)
  (funcall training environment)
  (cram-robot-memory:complete-learning nil)
  (roslisp-utilities::shutdown-ros)

  (roslisp-utilities::startup-ros)
  (funcall test environment)
  (roslisp-utilities::shutdown-ros))
