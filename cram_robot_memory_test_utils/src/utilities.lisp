(in-package :cram-robot-memory-test-utils)

(defun execute-training-and-test-functions(environment &key training test extend-existing-mlns)
  "This is a convinience function for executing test and training functions, either one after
   another or only one of them.
   For both test and training, ros is started initially and shutdown after the functions have
   been executed.
   The training function also calls the start-learning and complete learning function of the
   robot memory package.
   training and test must be functions accepting one arument (the environment, it will be
   usually :bullet)
   extend-existing-mlns is forwarded to the complete-learning function."
  (cond ((not (null training))
         (roslisp-utilities::startup-ros)
         (cram-robot-memory:start-learning)
         (funcall training environment)
         (cram-robot-memory:complete-learning extend-existing-mlns)
         (roslisp-utilities::shutdown-ros)))
  (cond ((not (null test))
         (roslisp-utilities::startup-ros)
         (funcall test environment)
         (roslisp-utilities::shutdown-ros))))
