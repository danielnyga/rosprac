(in-package :cram-robot-memory)

(defun get-failure-probability(task arguments &optional (goal-pattern " "))
  (let* ((config (get-rosmln-config nil))
         (task-evidence (task-information-to-evidence "Task"
                                                      :include-failure nil
                                                      :task-name task
                                                      :goal-context goal-pattern))
         (designator-evidence (get-multiple-designator-evidence arguments nil 0 0))
         (evidence (concatenate 'list task-evidence designator-evidence))
         (query (format nil "failure(Task, ?f)"))
         (query-result (rosmln:evidence-query config query evidence))
         (result (mapcar #'(lambda(pv) `(,(nth 0 pv)
                                         ,(let*((atom (nth 1 pv))
                                                (parts (cram-rosmln:split-atom atom))
                                                (failure (nth 2 parts))
                                                (end (- (length failure) 1))
                                                (symbol (subseq failure 1 end)))
                                            (if (equal symbol " ") nil symbol))))
                         query-result)))
    result))

(defun get-multiple-designator-evidence(remaining-args result desig-idx prop-idx) 
  (if (null remaining-args)
      result      
      (let*((key-designator-pair (car remaining-args))
            (key (nth 0 key-designator-pair))
            (designator (nth 1 key-designator-pair))
            (designator-props (get-designator-properties designator ""))
            (property-evidence (properties-to-evidence 
                                designator-props
                                (format nil "Desig~S" desig-idx)
                                prop-idx "Task" key))
            (combined-evidence (concatenate 'list property-evidence result))
            (new-start-idx (+ prop-idx (length designator-props))))
        (get-multiple-designator-evidence (cdr remaining-args) combined-evidence
                                          (+ desig-idx 1) new-start-idx))))
