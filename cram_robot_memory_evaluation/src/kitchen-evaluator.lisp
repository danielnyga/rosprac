(in-package :cram-robot-memory-evaluation)

(defun execute-training (kitchen-file-name
                         extend-existing-mlns
                         &optional (environment :bullet))
  (do-something-with-objects-in-kitchen
   kitchen-file-name :training environment #'train-grasping-objects extend-existing-mlns))

(defun execute-comparison-test(kitchen-file-name
                               location-file-name
                               &optional (environment :bullet))
  (do-something-with-objects-in-kitchen
    kitchen-file-name :test environment
    #'(lambda(kd tro teo) (test-grasping-objects
                           "comparison.csv"
                           (lambda(fun desig tsk ctx parm)
                             (complete-naively-with-next-on-failure
                              location-file-name fun desig tsk ctx parm)) 
                           kd
                           tro
                           teo))))

(defun execute-completion-test(kitchen-file-name &optional (environment :bullet))
  (do-something-with-objects-in-kitchen
    kitchen-file-name :test environment
    #'(lambda(kd tro teo) (test-grasping-objects
                           "completion.csv"
                           #'cram-robot-memory:complete-with-next-on-failure
                           kd
                           tro
                           teo))))

(defun do-something-with-objects-in-kitchen(kitchen-file-name
                                            scope
                                            environment
                                            operation
                                            &optional extend-existing-mlns)
  (funcall ; get rid of warning ...weakening keyword argument checking.
   'cram-robot-memory-test-utils:execute-training-and-test-functions
   environment
   :extend-existing-mlns extend-existing-mlns
   (ecase scope (:test :test) (:training :training))  
   #'(lambda(environment)
       (cram-robot-memory-test-utils:clean-and-spawn-kitchen-and-robot environment)
       (cram-robot-memory-test-utils:execute-in-environment
        environment
        (lambda()
          (let*((kitchen (eval (read-from-string (read-text-from-file kitchen-file-name))))
                (training-objects (mapcar #'(lambda(x) (nth 1 x)) kitchen))
                (test-objects (mapcar #'(lambda(x) (nth 2 x)) kitchen))
                (kitchen-dir (directory-namestring (pathname kitchen-file-name))))
            (spawn-objects kitchen environment)
            (funcall operation kitchen-dir training-objects test-objects)))))))

(defun read-text-from-file(file-name)
  (reduce #'(lambda(x y)(concatenate 'string x '(#\newline) y))
          (read-lines-from-file file-name)))

(defun read-lines-from-file(file-name)
  (labels ((read-file(stream &optional to-return)
             (let ((line (read-line stream nil 'eof nil)))
               (if (eq line 'eof)
                   (reverse to-return)
                   (read-file stream (cons line to-return))))))
    (with-open-file(file file-name)
      (read-file file))))

(defun read-csv(filename)
  (mapcar #'(lambda(l)(cram-robot-memory:split-recursively l ";"))
          (read-lines-from-file filename)))

(defun spawn-objects(pose-object-pairs environment &optional (index 0))
  (if (not (null pose-object-pairs))
      (let*((object (nth 1 (car pose-object-pairs)))
            (object-type (cram-designators:desig-prop-value object :type))
            (bullet-pose (car (car pose-object-pairs)))
            (object-name (format nil "~a-~a" (symbol-name object-type) index)))
        (cram-robot-memory-test-utils:spawn-object
         environment object-name object-type bullet-pose)
        (spawn-objects (cdr pose-object-pairs) environment (+ index 1)))))

(defun append-lines-to-file(file-name lines)
  (with-open-file(file
                  file-name
                  :direction :output
                  :if-exists :append
                  :if-does-not-exist :create)
    (mapcar #'(lambda(l) (write-line l file)) lines)))

(defun train-grasping-objects(kitchen-dir training-objects test-objects)
  (declare (ignore test-objects))
  (let*((failure-file-name (concatenate 'string kitchen-dir "/failures.txt"))
        (failure-information (train-grasping-objects-recursively training-objects))
        (failure-info-str (format nil "~a" failure-information))
        (failure-info-parts
          (if (null failure-information)
              ""
              (subseq failure-info-str 1 (- (length failure-info-str) 1)))))
    (append-lines-to-file failure-file-name `(,(format nil "~a" failure-info-parts)))))

(defun train-grasping-objects-recursively(objects &optional (failure-information nil))
  (if (null objects)
      failure-information
      (let*((object (car objects))
            (put-location (cram-designators:make-designator
                           :location '((:on "Cupboard") (:name "pancake_table"))))
            (updated-failure-information
              (cram-language:with-failure-handling
                  ((cram-language:plan-failure (e1)
                     (let*((object-name(cram-designators:desig-prop-value (car objects) :type))
                           (failure-name(type-of e1)))
                       (roslisp:ros-info (cram-robot-memory-evaluation) "Logging error ~a" e1)
                       (return (cons `(,object-name ,failure-name) failure-information)))))
                (cram-plan-library:achieve `(cram-plan-library:loc ,object ,put-location))
                failure-information)))
            (train-grasping-objects-recursively (cdr objects) updated-failure-information))))

(defun complete-naively-with-next-on-failure(locations-file do-sth-with designator
                                             &optional tsk ctxt parm)
  (declare (ignore tsk) (ignore ctxt) (ignore parm))
  (labels
      ((do-something(completed-designator)
         (cram-language:with-failure-handling
             ((cram-language:plan-failure (e) (declare (ignore e)) (return nil)))
           (funcall do-sth-with completed-designator)))
       (to-keyword(str) (symbol-value (intern (string-upcase str) "KEYWORD")))
       (complete-recursively(designator-description remaining-lines)
         (if (null remaining-lines)
             nil
             (let*((line (car remaining-lines))                         
                   (loc-desig-descr `((,(to-keyword (nth 0 line)) ,(nth 1 line))
                                      (,:name ,(nth 2 line))))
                   (loc-desig (cram-designators:make-designator :location loc-desig-descr))
                   (obj-desig-descr (cons `(:at ,loc-desig) designator-description))
                   (obj-desig (cram-designators:make-designator :object obj-desig-descr))
                   (result (do-something obj-desig)))
               (if (null result)
                   (complete-recursively designator-description (cdr remaining-lines))
                   result)))))
    (complete-recursively (cram-designators:description designator)
                        (read-csv locations-file))))

(defun test-grasping-objects(output-file-name completion-function kitchen-dir
                             training-objects test-objects)
  (append-lines-to-file (concatenate 'string kitchen-dir "/" output-file-name)
                        (mapcar #'(lambda(line)
                                    (reduce #'(lambda(x y) (format nil "~a;~a" x y)) line))
                                (test-grasping-objects-recursively completion-function
                                                                   training-objects
                                                                   test-objects
                                                                   nil))))

(defun test-grasping-objects-recursively(completion-func training-objects test-objects results)
  (if (null test-objects)
      results
      (let*((task-name "ACHIEVE")
            (goal-context "LOC ?OBJ ?LOC")
            (put-location (cram-designators:make-designator
                           :location '((:on "Cupboard") (:name "pancake_table"))))
            (params `(("?OBJ" ,(car training-objects)) ("?LOC" ,put-location)))
            (failure-probabilities
              (cram-robot-memory:get-failure-probability task-name params goal-context))
            (success-rate (car (car (remove-if #'(lambda(entry) (not (null (nth 1 entry))))
                                          failure-probabilities))))
            (str-success-rate (format nil "~,4f" success-rate)) 
            (object-type (cram-designators:desig-prop-value (car test-objects) :type))
            (tries 0)
            (comp-res(funcall completion-func
                      (lambda(object)
                        (cond ((= (length (cram-designators:description object))
                                  (length (cram-designators:description (car test-objects))))
                               (roslisp:ros-info (cram-robot-memory-evaluation)
                                                 "no completion found for ~a..." object) 
                               nil)
                              (t (setf tries (+ tries 1))
                                 (cram-plan-library:achieve
                                  `(cram-plan-library:loc ,object ,put-location)))))
                      (car test-objects)
                      task-name
                      goal-context
                      "?OBJ"))
            (success (not (not comp-res)))
            (result `(,object-type ,success ,str-success-rate ,tries)))
        (test-grasping-objects-recursively
          completion-func (cdr training-objects) (cdr test-objects) (cons result results)))))
