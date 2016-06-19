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
        (config (cram-rosmln:make-simple-config (if use-simplified-mln
                                                    "../learnt_mlns/designator.mln"
                                                    "../learnt_mlns/task_and_designator.mln")
                                                :fast-exact
                                                :first-order-logic
                                                :prac-grammar))
        (properties (get-designator-properties designator ""))
        (evidence (convert-to-evidence properties
                                       (not use-simplified-mln) task goal-ctxt goal-parm)))
    (roslisp:ros-info (cram-robot-memory) "completing designator ~a..." designator)
    (completion-result-target-fun-result
     (complete-key (make-completion-args :rosmln-config config
                                         :designator-properties properties
                                         :evidence evidence
                                         :seen-keys nil
                                         :tried-properties nil
                                         :target-function do-sth-with)))))

(defun test-completion() ;This is just for testing purposes - it can be removed...
	(let*((mug (cram-designators:make-designator :object `((:name "mug")))))
    (print (with-completed-designator #'(lambda(x) (declare (ignore x)) nil) mug)); 'achieve "OBJECT-IN-HAND ?OBJ"))
    t))

(defstruct completion-args
  rosmln-config
  designator-properties
  evidence
  seen-keys
  tried-properties
  target-function)

(defun update-completion-args
  (comp-args &key (rosmln-config (completion-args-rosmln-config comp-args))
                  (designator-properties (completion-args-designator-properties comp-args))
                  (evidence (completion-args-evidence comp-args))
                  (seen-keys (completion-args-seen-keys comp-args))
                  (tried-properties (completion-args-tried-properties comp-args))
                  (target-function (completion-args-target-function comp-args)))
  (make-completion-args :rosmln-config rosmln-config
                        :designator-properties designator-properties
                        :evidence evidence
                        :seen-keys seen-keys
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
        (seen-keys (completion-args-seen-keys comp-args))
        (best-key (get-argument-from-best-result results 1))
        (best-prob (car (car results)))
        (uniform-key-dist (every #'(lambda(e)(= (car e) best-prob)) results))
        (no-more-keys (or (< best-prob prob-threshold) uniform-key-dist)))
    (cond ((and no-more-keys (already-tried comp-args))
           (make-completion-result :tried-properties tried-props))
          (no-more-keys
           (roslisp:ros-info (cram-robot-memory) "completed ~a" desig-props)
           (make-completion-result :target-fun-result
                                   (funcall target-func (get-designator desig-props))
                                   :tried-properties (cons desig-props tried-props)))
          ((member-if #'(lambda(x) (equal x best-key)) seen-keys)
           (complete-key-rec comp-args (cdr results)))
          (t
           (recurse-if-result-null
            (complete-value comp-args best-key)
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
            (result (cram-rosmln:evidence-query conf query evidence))
            (second-best-prob (car (car (cdr result))))
            (seen-keys (completion-args-seen-keys comp-args))
            (seen-upd (if (< second-best-prob prob-threshold)
                          (cons key seen-keys)
                          seen-keys)))
        (complete-val-rec (update-completion-args comp-args
                                                  :evidence evidence
                                                  :seen-keys seen-upd)
                          result key))))

(defun complete-val-rec(comp-args results key)
  (let*((desig-props (completion-args-designator-properties comp-args))
        (evidence (completion-args-evidence comp-args))
        (tried-props (completion-args-tried-properties comp-args))
        (target-func (completion-args-target-function comp-args))
        (seen-keys (completion-args-seen-keys comp-args))
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
                                                  :evidence updated-ev
                                                  :seen-keys (cons key seen-keys)))
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
               (properties-to-evidence properties "Designator"
                                       (if include-task-info "Task" nil)
                                       goal-param)
               (if include-task-info 
                   (task-information-to-evidence "Task" t
                                                 (if (null task) nil
                                                     (symbol-name task))
                                                 goal-context)
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
