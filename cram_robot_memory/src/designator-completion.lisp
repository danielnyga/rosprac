(in-package :cram-robot-memory)

(defconstant prob-threshold 0.00001)

(defun complete-with-next-on-failure(do-sth-with designator &optional task goal-ctxt goal-parm)
  (car (with-completed-designator
         #'(lambda(d)
           (cram-language:with-failure-handling
               ((cram-language:plan-failure (e)
                  (roslisp:ros-info (cram-robot-memory-demo)
                                    "encountered an ~a error - retrying..." e)
                  (return nil)))
             `(,(funcall do-sth-with d))))
         designator task goal-ctxt goal-parm)))                       

(defun with-completed-designator(do-sth-with designator &optional task goal-ctxt goal-parm) 
  (let*((use-simplified-mln (and (null task) (null goal-ctxt) (null goal-parm)))
        (config (get-rosmln-config use-simplified-mln))
        (properties (get-designator-properties designator ""))
        (evidence (convert-to-evidence properties
                                       (not use-simplified-mln) task goal-ctxt goal-parm)))
    (roslisp:ros-info (cram-robot-memory) "completing designator ~a..." designator)
    (completion-result-target-fun-result
     (complete-key (make-completion-args :rosmln-config config
                                         :designator-properties properties
                                         :evidence evidence
                                         :number-of-probable-keys nil
                                         :tried-properties nil
                                         :target-function do-sth-with)))))

(defun test-completion() ;This is just for testing purposes - it can be removed...
	(let*((mug (cram-designators:make-designator :object `((:name "mug")))))
    (print (with-completed-designator #'(lambda(x) (declare (ignore x)) nil)
             mug "ACHIEVE" "OBJECT-IN-HAND ?OBJ" "?OBJ"))
    t))

(defstruct completion-args
  rosmln-config
  designator-properties
  evidence
  number-of-probable-keys
  tried-properties
  target-function)

(defun update-completion-args
  (comp-args &key (rosmln-config (completion-args-rosmln-config comp-args))
                  (designator-properties (completion-args-designator-properties comp-args))
                  (evidence (completion-args-evidence comp-args))
                  (number-of-probable-keys (completion-args-number-of-probable-keys comp-args))
                  (tried-properties (completion-args-tried-properties comp-args))
                  (target-function (completion-args-target-function comp-args)))
  (make-completion-args :rosmln-config rosmln-config
                        :designator-properties designator-properties
                        :evidence evidence
                        :number-of-probable-keys number-of-probable-keys
                        :tried-properties tried-properties
                        :target-function target-function))

(defstruct completion-result
  (target-fun-result nil)
  (tried-properties nil)
  (call-in-last-rec nil))

(defun update-completion-result
  (comp-result &key (target-fun-result (completion-result-target-fun-result comp-result))
                    (tried-properties (completion-result-tried-properties comp-result))
                    (call-in-last-rec (completion-result-call-in-last-rec comp-result)))
  (make-completion-result :target-fun-result target-fun-result
                          :tried-properties tried-properties
                          :call-in-last-rec call-in-last-rec))

(defun complete-key(comp-args)
  (let*((prop-idx (length (completion-args-designator-properties comp-args)))
        (prop-atom (format nil "designatorProperty(Prop~D, Designator)" prop-idx))
        (evidence (cons prop-atom (completion-args-evidence comp-args)))
        (query (format nil "propertyKey(Prop~D, ?k)" prop-idx))
        (result (cram-rosmln:evidence-query (completion-args-rosmln-config comp-args)
                                            query
                                            evidence)))
    (complete-key-rec (update-completion-args comp-args :evidence evidence) result)))

(defun complete-key-rec(comp-args results)
  (let*((tried-props (completion-args-tried-properties comp-args))
        (desig-props (completion-args-designator-properties comp-args))
        (target-func (completion-args-target-function comp-args))
        (number-of-keys-before (completion-args-number-of-probable-keys comp-args))
        (best-key (get-argument-from-best-result results 1))
        (best-prob (car (car results)))
        (probable-keys (remove-if #'(lambda(result) (< (car result) prob-threshold)) results))
        (more-keys-than-before (and (not (null number-of-keys-before))
                                    (> (length probable-keys) number-of-keys-before)))
        (uniform-key-dist (every #'(lambda(e)(= (car e) best-prob)) results))
        (no-more-keys (or (< best-prob prob-threshold) uniform-key-dist)))
    (cond ((and (or no-more-keys more-keys-than-before) (already-tried comp-args))
           (make-completion-result :tried-properties tried-props))
          ((or no-more-keys more-keys-than-before)
           (roslisp:ros-info (cram-robot-memory) "completed ~a" desig-props)
           (make-completion-result :target-fun-result
                                   (funcall target-func (get-designator desig-props))
                                   :tried-properties (cons desig-props tried-props)))
          (t
           (recurse-if-result-null
            (complete-value (update-completion-args
                             comp-args :number-of-probable-keys (length probable-keys))
                            best-key)
            #'(lambda(result)
                (if (completion-result-call-in-last-rec result) ;retuned from a direct call
                    (update-completion-result result :call-in-last-rec nil)
                    (complete-key-rec
                     (update-completion-args
                      comp-args
                      :tried-properties (completion-result-tried-properties result))
                     (cdr results)))))))))

(defun complete-value(comp-args key)
  (if (already-tried comp-args key)
      (make-completion-result :tried-properties (completion-args-tried-properties comp-args))
      (let*((prop-idx (length (completion-args-designator-properties comp-args)))
            (best-key-atom (format nil "propertyKey(Prop~D,~S)" prop-idx key))
            (evidence (cons best-key-atom (completion-args-evidence comp-args)))
            (query (format nil "propertyValue(Prop~D, ?v)" prop-idx))
            (conf (completion-args-rosmln-config comp-args))
            (result (cram-rosmln:evidence-query conf query evidence)))
        (complete-val-rec (update-completion-args comp-args
                                                  :evidence evidence)
                          result key))))

(defun complete-val-rec(comp-args results key)
  (let*((desig-props (completion-args-designator-properties comp-args))
        (evidence (completion-args-evidence comp-args))
        (tried-props (completion-args-tried-properties comp-args))
        (target-func (completion-args-target-function comp-args))
        (best-val (get-argument-from-best-result results 1))
        (prop-idx (length desig-props))
        (best-prob (car (car results)))
        (uniform-val-dist (every #'(lambda(e)(= (car e) best-prob)) results))
        (kv-pair `(,key ,best-val))
        (updated-props (cons kv-pair desig-props))
        (updated-ev (cons (format nil "propertyValue(Prop~D,~S)" prop-idx best-val) evidence))
        (no-more-values (or (< best-prob prob-threshold) uniform-val-dist)))
    (cond ((and no-more-values (already-tried comp-args))
           (make-completion-result :tried-properties tried-props))
          (no-more-values
           (roslisp:ros-info (cram-robot-memory) "completed ~a" desig-props)
           (make-completion-result :target-fun-result
                                   (funcall target-func (get-designator desig-props))
                                   :tried-properties (cons desig-props tried-props)
                                   :call-in-last-rec t))
          ((member-if #'(lambda(x) (equal x kv-pair)) desig-props)
           (complete-val-rec comp-args (cdr results) key))
          ((equal best-val "<hard coded pose>")
           (make-completion-result :tried-properties tried-props))
          (t
           (roslisp:ros-info (cram-robot-memory) "updated: ~a" updated-props)
           (recurse-if-result-null
            (complete-key (update-completion-args comp-args
                                                  :designator-properties updated-props
                                                  :evidence updated-ev))
            #'(lambda(result)
                (complete-val-rec
                 (update-completion-args
                  comp-args
                  :tried-properties (completion-result-tried-properties result))
                 (cdr results) key)))))))

(defun already-tried(comp-args &optional key)
  (let*((tried-props (completion-args-tried-properties comp-args))
        (desig-props (completion-args-designator-properties comp-args))
        (tries (remove-if (lambda(w) (not (subsetp desig-props w :test #'equal))) tried-props)))
    (if (null key)
        (not (null tries))
        (some #'(lambda(w) (some #'(lambda(kv) (equal (car kv) key)) w)) tries))))

(defun convert-to-evidence(properties include-task-info
                           &optional task goal-context goal-param )
  (concatenate 'list
               (properties-to-evidence properties "Designator" 0
                                       (if include-task-info "Task" nil)
                                       goal-param)
               (if include-task-info 
                   (task-information-to-evidence
                    "Task"
                    :include-failure t
                    :failure nil
                    :task-name task
                    :goal-context goal-context)
                   nil)))
                                                                      
(defun get-argument-from-best-result(results argument-index)
  (let*((best-result (cram-rosmln:split-atom (car (last (car results)))))
        (argument (nth (+ 1 argument-index) best-result)))
    (if (> (length argument) 2)
        (subseq argument 1 (- (length argument) 1))
        "")))

(defun recurse-if-result-null(result recursion-function)
  (if (null (completion-result-target-fun-result result))
      (funcall recursion-function result)
      result))
