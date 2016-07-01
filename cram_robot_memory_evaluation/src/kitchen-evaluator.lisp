(in-package :cram-robot-memory-evaluation)

(defun handle-objects-in-kitchen(kitchen-file-name
                                 extend-existing-mlns
                                 &optional (environment :bullet))
  (cram-robot-memory-test-utils:execute-training-and-test-functions
   environment
   :extend-existing-mlns extend-existing-mlns
   :training
   #'(lambda(environment)
       (cram-robot-memory-test-utils:clean-and-spawn-kitchen-and-robot environment)
       (cram-robot-memory-test-utils:execute-in-environment
        environment
        (lambda()
          (with-open-file(kitchen-file kitchen-file-name)
            (let*((kitchen (eval (read-from-string (read-file kitchen-file))))
                  (kitchen-objects (mapcar #'(lambda(x) (nth 1 x)) kitchen))
                  (kitchen-dir (directory-namestring (pathname kitchen-file-name)))
                  (failure-file-name (concatenate 'string kitchen-dir "/failures.txt")))
              (spawn-objects kitchen environment)
              (grasp-objects kitchen-objects failure-file-name))))))))

(defun read-file(stream &optional (to-return ""))
  (let ((line (read-line stream nil 'eof nil))
        (separator (if (equal to-return "") "" '(#\newline))))
    (if (eq line 'eof)
        to-return
        (read-file stream (concatenate 'string to-return separator line)))))

(defun spawn-objects(pose-object-pairs environment &optional (index 0))
  (if (not (null pose-object-pairs))
      (let*((object (nth 1 (car pose-object-pairs)))
            (object-type (cram-designators:desig-prop-value object :type))
            (bullet-pose (car (car pose-object-pairs)))
            (object-name (format nil "~a-~a" (symbol-name object-type) index)))
        (cram-robot-memory-test-utils:spawn-object
         environment object-name object-type bullet-pose)
        (spawn-objects (cdr pose-object-pairs) environment (+ index 1)))))

(defun grasp-objects(objects failure-file-name)
  (let*((failure-information (grasp-objects-recursively objects))
        (failure-info-str (format nil "~a" failure-information))
        (failure-info-parts
          (if (null failure-information)
              ""
              (subseq failure-info-str 1 (- (length failure-info-str) 1)))))
    (with-open-file(failure-file
                    failure-file-name
                    :direction :output
                    :if-exists :append
                    :if-does-not-exist :create)
      (write-line (format nil "~a" failure-info-parts) failure-file))))

(defun grasp-objects-recursively(objects &optional (failure-information nil))
  (if (null objects)
      failure-information
      (cram-language:with-failure-handling
          ((cram-language:plan-failure (e1)
             (let*((object-name (cram-designators:desig-prop-value (car objects) :type))
                   (failure-name (type-of e1))
                   (updated-failures (cons `(,object-name ,failure-name) failure-information)))
               (roslisp:ros-info (cram-robot-memory-evaluation) "Logging error ~a" e1)
               (return (grasp-objects-recursively (cdr objects) updated-failures)))))      
        (let*((object (car objects))
              (perceived (cram-plan-library:perceive-object 'cram-plan-library:a object))
              (put-location (cram-designators:make-designator
                             :location '((:on "Cupboard") (:name "pancake_table")))))
          (unless (cram-designators:desig-equal object perceived)
            (cram-designators:equate object perceived))
          (cram-plan-library:achieve `(cram-plan-library:loc ,object ,put-location))
          (grasp-objects-recursively (cdr objects) failure-information)))))
