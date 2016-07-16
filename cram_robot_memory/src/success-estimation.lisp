(in-package :cram-robot-memory)

(defun get-failure-probability(task arguments &optional (goal-pattern " "))
  (mapcar #'(lambda(prob-failure) (if (equal (nth 1 prob-failure) " ")
                                      `(,(nth 0 prob-failure) nil)
                                      prob-failure))
          (execute-general-query "failure(Task, ?f)"
                                 :result-index 1
                                 :task-name task
                                 :goal-pattern goal-pattern
                                 :arguments arguments
                                 :include-success nil)))
