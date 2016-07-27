(in-package :cram-robot-memory)

(defun execute-general-query(query &key result-index
                                        task-name
                                        goal-pattern
                                        arguments
                                        include-success
                                        evidence)

  "Queries the MLN.
   query is the query as a string, e.g. propertyValue(Prop1, ?v).
   task is the name of a cram plan, e.g. ACHIEVE to be used as evidence.
   goal-pattern is a goal-pattern, e.g. OBJECT-IN-HAND ?OBJ, to be used as evidence
   arguments is a list containing arguments for the task to be used as evidence.
   More specifically, one argument is represent as a list where the first
   entry is the argument name, e.g. ?OBJ and the second entry is the
   argument itself, e.g. a designator.
   evidence is additional evidence in form of a string which is appended to the
   query database.
   The return value is a list representing the probability distribution.
   This list contains other lists with the first entry being the probability
   and the second entry being the query result. An example is:
   ((0.5 \"propertyValue(Prop1,\\\"Foo\\\")\") (0.5 \"propertyValue(Prop1,\\\"Bar\\\")\")) 
   The result-index can be used to show only one argument if the results are ground atoms.
   In the example, a result-index of 1 would lead to ((0.5 \"Foo\") (0.5 \"Bar\"))"  
  (let*((config (get-rosmln-config nil))
        (task-evidence (if (null task-name)
                           nil
                           (task-information-to-evidence "Task"
                                                         :include-failure include-success
                                                         :failure " "
                                                         :task-name task-name
                                                         :goal-context goal-pattern)))
        (designator-evidence (get-multiple-designator-evidence arguments nil 0 0))
        (evidence (concatenate 'list task-evidence designator-evidence evidence))
        (query-result (rosmln:evidence-query config query evidence))
        (filtered-result (remove-if (lambda(x) (= 0.0 (car x))) query-result)))
    (if (null result-index)
        filtered-result
        (mapcar #'(lambda(pv) `(,(nth 0 pv)
                                ,(let*((atom (nth 1 pv))
                                       (parts (cram-rosmln:split-atom atom))
                                       (part (nth (+ 1 result-index) parts))
                                       (end (- (length part) 1)))
                                   (subseq part 1 end))))
                filtered-result))))
