(in-package :cram-robot-memory)

(defun get-failure-probability(task arguments &optional (goal-pattern " "))
  "Calculates the failure probability of task with the goal pattern goal-pattern
   and with the arguments arguments.
   task is the name of a cram plan, e.g. ACHIEVE
   goal-pattern is a goal-pattern, e.g. OBJECT-IN-HAND ?OBJ
   arguments is a list containing arguments for the task.
   More specifically, one argument is represent as a list where the first
   entry is the argument name, e.g. ?OBJ and the second entry is the
   argument itself, e.g. a designator.
   The return value is a list representing the probability distribution.
   This list contains other lists with the first entry being the probability
   and the second entry being the failure name or nil for the success value"
  (mapcar #'(lambda(prob-failure) (if (equal (nth 1 prob-failure) " ")
                                      `(,(nth 0 prob-failure) nil)
                                      prob-failure))
          (execute-general-query "failure(Task, ?f)"
                                 :result-index 1
                                 :task-name task
                                 :goal-pattern goal-pattern
                                 :arguments arguments
                                 :include-success nil)))
