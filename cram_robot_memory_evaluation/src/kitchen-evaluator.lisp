(in-package :cram-robot-memory-evaluation)

(defun execute-training (kitchen-file-name
                         extend-existing-mlns
                         &optional (environment :bullet))
  (do-something-with-objects-in-kitchen
   kitchen-file-name "training.csv" :training environment t
    #'train-moving-objects extend-existing-mlns))

(defun execute-naive-completion-test(kitchen-file-name
                                     location-file-name
                                     &optional (environment :bullet))
  (do-something-with-objects-in-kitchen
    kitchen-file-name "naive_completion.csv" :test environment t
    #'(lambda(training-object test-object)
        (declare (ignore training-object))
        (evaluate-completion-function test-object
                                      (lambda(fun desig tsk ctx parm)
                                        (complete-naively-with-next-on-failure
                                         location-file-name fun desig tsk ctx parm))
                                      #'move-to-pancake-table))))

(defun execute-mln-completion-test(kitchen-file-name &optional (environment :bullet))
  (do-something-with-objects-in-kitchen
    kitchen-file-name "mln_completion.csv" :test environment t
    #'(lambda(training-object test-object)
        (declare (ignore training-object))
        (evaluate-completion-function test-object                                    
                                      #'cram-robot-memory:complete-with-next-on-failure
                                      #'move-to-pancake-table))))

(defun execute-theoretical-test(kitchen-file-name
                                location-file-name
                                &optional (environment :bullet))
  (do-something-with-objects-in-kitchen
    kitchen-file-name "theoretical.csv" :test environment nil
    #'(lambda(training-object test-object)
        (test-theoretically location-file-name
                            training-object
                            test-object))))

(defun execute-informed-test(kitchen-file-name
                             &optional (environment :bullet))
  (do-something-with-objects-in-kitchen
    kitchen-file-name "informed.csv" :test environment t
    #'(lambda(training-object test-object)
        (evaluate-completion-function test-object
                                      #'(lambda(fun desig task ctx parm)
                                          (declare (ignore task desig ctx parm))
                                          (cram-language:with-failure-handling
                                              ((cram-language:plan-failure (e)
                                                 (declare (ignore e))
                                                 (return nil)))
                                            (funcall fun training-object)))
                                      #'move-to-pancake-table))))

(defun do-something-with-objects-in-kitchen(kitchen-file-name
                                            output-file-name
                                            scope
                                            environment                                            
                                            spawn-objects
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
                (kitchen-dir (directory-namestring (pathname kitchen-file-name))))
            (if spawn-objects
                (spawn-objects kitchen environment))
            (write-list-to-csv-file
             kitchen-dir
             output-file-name
             (reduce #'(lambda(result kitchen)
                         (cons (funcall operation (second kitchen) (third kitchen)) result))
                     kitchen
                     :initial-value nil))))))))
             
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

(defun write-list-to-csv-file(dirname filename list)
  (append-lines-to-file (concatenate 'string dirname "/" filename)
                        (mapcar #'(lambda(line)
                                    (reduce #'(lambda(x y) (format nil "~a;~a" x y)) line))
                                list)))

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

(defun train-moving-objects(train-object test-object)
  (declare (ignore test-object))
  (let ((object-name (format nil "~a" (cram-designators:desig-prop-value train-object :type)))
        (location (format nil "~a" (cram-designators:description
                                   (cram-designators:desig-prop-value train-object :at)))))
    (cram-language:with-failure-handling
        ((cram-language:plan-failure (e1)
           (roslisp:ros-info (cram-robot-memory-evaluation) "Logging error ~a" e1)
           (return `(,object-name ,location , (type-of e1)))))
      (move-to-pancake-table train-object)
      `(,object-name ,location nil))))

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

(defun test-moving-objects(output-file-name completion-function kitchen-dir
                             training-objects test-objects)
  (declare (ignore training-objects))
  (write-list-to-csv-file
   kitchen-dir
   output-file-name
   (reduce #'(lambda(result test-object)
               (cons 
                (evaluate-completion-function
                 test-object
                 completion-function
                 #'move-to-pancake-table)
                result))
           test-objects
           :initial-value nil)))
  
(defun test-theoretically(location-filename training-object test-object)
  (let*((real-object-location (cram-designators:desig-prop-value training-object :at))
        (object-loc-str (format nil "~a" (cram-designators:description real-object-location)))
        (test-object-type (cram-designators:desig-prop-value test-object :type))
        (training-object-type (cram-designators:desig-prop-value training-object :type))
        (completion-result-check
          #'(lambda (designator)
              (if (not (subsetp
                        (cram-designators:description real-object-location)
                        (cram-designators:description
                         (cram-designators:desig-prop-value designator :at))
                        :test #'equal))
                  (cram-language:fail 'cram-plan-failures:object-not-found)
                  t)))
        (naive-result (evaluate-completion-function
                       test-object
                       #'(lambda(fun desig tsk ctx parm)
                           (complete-naively-with-next-on-failure
                            location-filename fun desig tsk ctx parm))                             
                       completion-result-check))
        (mln-result (evaluate-completion-function
                     test-object
                     #'cram-robot-memory:complete-with-next-on-failure
                     completion-result-check))
        (put-location (cram-designators:make-designator
                       :location '((:on "Cupboard") (:name "pancake_table"))))
        (success-params `(("?OBJ" ,training-object) ("?LOC" ,put-location)))
        (failure-probabilities
          (cram-robot-memory:get-failure-probability "ACHIEVE" success-params "LOC ?OBJ ?LOC"))
        (success-rate (car (car (remove-if #'(lambda(entry) (not (null (second entry))))
                                           failure-probabilities))))
        (str-success-rate (format nil "~,4f" (if (null success-rate) 0.0 success-rate)))

        (location-params `(("?OBJ-DESIG" ,(cram-designators:make-designator
                                          :object `((:at ,real-object-location))))))
        (object-type-probabilities (cram-robot-memory:execute-general-query
                                    "propertyValue(Obj, ?v)"
                                    :result-index 1
                                    :task-name "PERCEIVE-OBJECT"
                                    :goal-pattern "A ?OBJ-DESIG"
                                    :arguments location-params
                                    :include-success t
                                    :evidence '("propertyKey(Obj,\"object.TYPE\")")))
        (location-pos (position (symbol-name test-object-type) object-type-probabilities
                                :test #'(lambda(x y) (equal x (second y)))))
        (location-rank (if (null location-pos) nil (+ location-pos 1))))
    (assert (equal test-object-type training-object-type))
    (concatenate 'list `(,test-object-type ,object-loc-str ,str-success-rate ,location-rank)
                 (cdr naive-result) (cdr mln-result))))
  
(defun evaluate-completion-function(designator completion-function evaluation-function)
  (let*((object-type (cram-designators:desig-prop-value designator :type))
        (tries 0)
        (comp-res(funcall completion-function
                          (lambda(object)
                            (cond ((= (length (cram-designators:description object))
                                      (length (cram-designators:description designator)))
                                   (roslisp:ros-info (cram-robot-memory-evaluation)
                                                     "no completion found for ~a..." object) 
                                   nil)
                                  (t (setf tries (+ tries 1))
                                     (funcall evaluation-function object))))
                          designator
                          "ACHIEVE"
                          "LOC ?OBJ ?LOC"
                          "?OBJ"))
        (success (not (not comp-res))))
    `(,object-type ,success ,tries)))

(defun move-to-pancake-table(object)
  (cram-plan-library:achieve `(cram-plan-library:loc
                               ,object
                               ,(cram-designators:make-designator
                                 :location '((:on "Cupboard") (:name "pancake_table"))))))
