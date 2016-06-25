(in-package :cram-robot-memory-evaluation)

(defun handle-objects-in-kitchen(kitchen-file-name &optional (environment :bullet))
  (cram-robot-memory-demo::execute-training-and-test-functions
   environment
   :training
   #'(lambda(environment)
       (print (concatenate 'string "Operating on kitchen " kitchen-file-name))
       (cram-robot-memory-demo::clean-and-spawn-kitchen-and-robot environment)
       (cram-robot-memory-demo::execute-in-environment
        environment
        (lambda()
          (with-open-file(kitchen-file kitchen-file-name)
            (let*((kitchen (eval (read-from-string (read-file kitchen-file))))
                  (kitchen-dir (directory-namestring (pathname kitchen-file-name)))
                  (failure-file-name (concatenate 'string kitchen-dir "/failures.txt")))
              (spawn-objects kitchen environment)
              (grasp-objects kitchen failure-file-name))))))))

(defun read-file(stream &optional (to-return ""))
  (let ((line (read-line stream nil 'eof nil))
        (separator (if (equal to-return "") "" '(#\newline))))
    (if (eq line 'eof)
        to-return
        (read-file stream (concatenate 'string to-return separator line)))))

(defun spawn-objects(objects environment &optional (index 0))
  (if (not (null objects))
      (let*((object-type (cram-designators:desig-prop-value (car objects) :type))
            (object-loc (cram-designators:desig-prop-value (car objects) :at))
            (object-pose (cram-designators:reference object-loc))
            (position (cl-transforms:origin object-pose))
            (orientation (cl-transforms:orientation object-pose))
            (pos `(,(cl-transforms:x position)
                   ,(cl-transforms:y position)
                   ,(+ (cl-transforms:z position) 0.125))) ; sampled poses are too low...
            (or `(,(cl-transforms:x orientation)
                  ,(cl-transforms:y orientation)
                  ,(cl-transforms:z orientation)
                  ,(cl-transforms:w orientation)))
            (bullet-pose `(,pos ,or))
            (object-name (format nil "~a-~a" (symbol-name object-type) index))
            (object-symb-name (symbol-value (intern object-name "KEYWORD"))))
        (cram-robot-memory-demo::spawn-object
         environment object-symb-name object-type bullet-pose)
        (spawn-objects (cdr objects) environment (+ index 1)))))

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
          ((cram-language:plan-failure (e)
             (let*((object-name (cram-designators:desig-prop-value (car objects) :type))
                   (failure-name (type-of e))
                   (updated-failures (cons `(,object-name ,failure-name) failure-information)))
               (return (grasp-objects-recursively (cdr objects) updated-failures)))))
        (cram-plan-library:achieve `(cram-plan-library:object-in-hand ,(car objects)))
        (grasp-objects-recursively (cdr objects) failure-information))))
