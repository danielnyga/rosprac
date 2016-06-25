(in-package :cram-robot-memory-demo)

(defun execute-success-estimation-demo(&optional (environment :bullet))
  (execute-training-and-test-functions environment
                                       :training #'create-success-estimation-training-data
                                       :test #'execute-success-estimation-test))

(defun create-success-estimation-training-data(environment)
  (clean-and-spawn-kitchen-and-robot environment)
  ;Scenario: Grasp an expensive object (e.g. an expensive mug or plate...)
  (spawn-object environment 'fork-1 :fork '((-0.9  0.0 0.75) (0 0 1 1)))
  (spawn-object environment 'mug-1 :mug '((-0.9 1.5 0.9) (0 0 0 1)))
  (execute-in-environment
   environment
   (lambda()
     ;the following grasp tasks will fail:
     (pick-and-place :fork "CounterTop" "kitchen_sink_block_counter_top")                               
     (pick-and-place :fork "CounterTop" "kitchen_sink_block_counter_top")                               
     (pick-and-place :mug "CounterTop" "kitchen_sink_block_counter_top")
     ;the following grasp tasks will succeed:
     (pick-and-place :fork "Cupboard" "pancake_table")
     (pick-and-place :mug "CounterTop" "kitchen_island_counter_top"))))

(defun execute-success-estimation-test(environment)
  (declare (ignore environment))
  (let* ((counter-top (cram-designators:make-designator
                       :location
                       '((:on "CounterTop") (:name "kitchen_island_counter_top"))))
         (mug (cram-designators:make-designator :object `((:type :mug) (:at ,counter-top))))
         (fork (cram-designators:make-designator :object `((:type :fork) (:at ,counter-top))))
         (mug-prob (cram-robot-memory:get-failure-probability "ACHIEVE"
                                                              `(("?OBJ" ,mug))
                                                              "OBJECT-IN-HAND ?OBJ"))
         (fork-prob (cram-robot-memory:get-failure-probability "ACHIEVE"
                                                               `(("?OBJ" ,fork))
                                                               "OBJECT-IN-HAND ?OBJ")))
    (roslisp:ros-info
     (cram-robot-memory-demo)
     "Error probabilities for grasping a mug on kitchen_island_counter_top:~C~a"
     #\newline mug-prob)
    (roslisp:ros-info
     (cram-robot-memory-demo)
     "Error probabilities for grasping a fork on kitchen_island_counter_top:~C~a"
     #\newline fork-prob)
    (assert (result-has-probability mug-prob nil 1.0))
    (assert (result-has-probability fork-prob nil 0.33))))

(cram-language:def-cram-function pick-and-place(obj-type loc-on loc-name)
  (cram-language:with-failure-handling
      ((cram-language:plan-failure (e)
         (roslisp:ros-info (cram-robot-memory-demo)
                           "grasping failed: encountered ~a failure" e)
         (return nil)))
    (cram-language-designator-support:with-designators
        ((obj-location :location `((:on ,loc-on) (:name ,loc-name)))
         (obj :object `((:type ,obj-type) (:at ,obj-location))))
      (let ((picked (cram-plan-library:achieve `(cram-plan-library:object-in-hand ,obj))))
        (cram-plan-library:achieve `(cram-plan-library:object-placed-at ,picked ,
                                                                        obj-location))))))
(defun result-has-probability(result element probability)
  (if (remove-if #'(lambda(pair) (not (and (equal (nth 1 pair) element)
                                           (compare-floats (nth 0 pair) probability))))
             result) t nil))

(defun compare-floats(f1 f2)
  (< (abs (- f1 f2)) 0.01))
