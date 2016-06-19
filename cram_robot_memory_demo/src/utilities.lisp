(in-package :cram-robot-memory-demo)

(defun execute-training-and-test-functions(environment training test)
  (roslisp-utilities::startup-ros)
  (clean-and-spawn-kitchen-and-robot environment)
  (cram-robot-memory:start-learning)
  (cram-language:top-level
    (execute-in-environment environment #'(lambda() (funcall training environment))))
  (cram-robot-memory:complete-learning nil)
  (roslisp-utilities::shutdown-ros)

  (roslisp-utilities::startup-ros)
  (clean-and-spawn-kitchen-and-robot environment)
  (cram-language:top-level
    (execute-in-environment environment #'(lambda() (funcall test environment))))
  (roslisp-utilities::shutdown-ros))
