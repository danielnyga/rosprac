(in-package :cram-robot-memory)

(defun execute-general-query(query &key result-index
                                        task-name
                                        goal-pattern
                                        arguments
                                        include-success
                                        evidence)
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
