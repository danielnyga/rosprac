(in-package :cram-robot-memory-demo)

(defun execute-colloquium-demo(&optional (environment :bullet))
  (roslisp:set-debug-levels nil :fatal)
  (roslisp:set-debug-levels colloquium :info)
  (cram-robot-memory-test-utils:execute-training-and-test-functions
   environment
   :training #'spawn-objects-and-execute-training-plans
   :test #'spawn-objects-and-execute-test-plans) 
)

(defun spawn-objects-and-execute-training-plans(environment)
  (cram-robot-memory-test-utils:clean-and-spawn-kitchen-and-robot environment)
  (cram-robot-memory-test-utils:spawn-object
   environment "knife-1" :knife '((-1.3 1.7 0.95) (0 0 0 1)))
  (cram-robot-memory-test-utils:spawn-object
   environment "knife-2" :knife '((1.4 0.0 0.95) (0 0 0 1)))
  (cram-robot-memory-test-utils:spawn-object
   environment "mug-1" :mug '((1.4 -0.5 0.95) (0 0 0 1)))
  (cram-robot-memory-test-utils:execute-in-environment
   environment
   #'(lambda()
       (execute-training-plans))))

(defun spawn-objects-and-execute-test-plans(environment)
  (cram-robot-memory-test-utils:clean-and-spawn-kitchen-and-robot environment)
  (cram-robot-memory-test-utils:spawn-object
   environment "knife-1" :knife '((-1.3 1.7 0.95) (0 0 0 1)))
  (cram-robot-memory-test-utils:execute-in-environment
   environment
   #'(lambda()
       (execute-test-plans))))

(defun achieve-loc-pancake-table(object)
  (cpl:with-failure-handling
      ((cpl:plan-failure (f)
         (declare (ignore f))
         (roslisp:ros-info colloquium "failed")
         (return)))
    (cpl-desig-supp:with-designators
        ((on-pancake-table :location
                           '((:on "Cupboard")
                             (:name "pancake_table"))))
      (plan-lib:achieve `(plan-lib:loc ,object ,on-pancake-table))
      (roslisp:ros-info colloquium "success"))))

(defun execute-training-plans()
  (roslisp:ros-info colloquium "training...")
  (roslisp:ros-info colloquium "move a knife from the kitchen_sink_block to the pancake_table")
  (achieve-loc-pancake-table (desig:make-designator
                              :object `((:type :knife)
                                        (:at ,(desig:make-designator
                                               :location
                                               '((:on "Cupboard")
                                                 (:name "kitchen_sink_block")))))))
  (roslisp:ros-info colloquium "move a knife from kitchen_island to the pancake_table")
  (achieve-loc-pancake-table (desig:make-designator
                              :object `((:type :knife)
                                        (:at ,(desig:make-designator
                                               :location
                                               '((:on "Cupboard")
                                                 (:name "kitchen_island")))))))
  (roslisp:ros-info colloquium "move a knife from the kitchen_sink_block to the pancake_table")
  (achieve-loc-pancake-table (desig:make-designator
                              :object `((:type :knife)
                                        (:at ,(desig:make-designator
                                               :location
                                               '((:on "Cupboard")
                                                 (:name "kitchen_sink_block")))))))
  (roslisp:ros-info colloquium "move a mug from the kitchen_sink_block to the pancake_table")
  (achieve-loc-pancake-table (desig:make-designator
                              :object `((:type :mug)
                                        (:at ,(desig:make-designator
                                               :location
                                               '((:on "Cupboard")
                                                 (:name "kitchen_sink_block")))))))
  (roslisp:ros-info colloquium "finished training!")
  (roslisp:ros-info colloquium "learning..."))

(defun execute-test-plans()
  (roslisp:ros-info colloquium "finished learning!")
  (roslisp:ros-info
   colloquium
   "objects on the kitchen_sink_block:~C~a"
   #\linefeed
   (cram-robot-memory:execute-general-query
    "propertyValue(Obj, ?v)"
    :result-index 1
    :task-name "PERCEIVE-OBJECT"
    :goal-pattern "A ?OBJ-DESIG"
    :include-success t
    :evidence '("propertyKey(Obj,\"object.TYPE\")")
    :arguments `(("?OBJ-DESIG"
                  ,(desig:make-designator
                    :object
                    `((:at ,(desig:make-designator
                             :location '((:on "Cupboard")
                                         (:name "kitchen_sink_block"))))))))))
  (break)
  (roslisp:ros-info
   colloquium
   "failure probabilities for moving a knife to the pancake_table:~C~a"
   #\linefeed
   (cram-robot-memory:get-failure-probability
    "ACHIEVE"
    `(("?OBJ",(desig:make-designator :object `((:type :knife)))
       "?LOC",(desig:make-designator
               :location `((:on "Cupboard") (:name "pancake_table")))))
    "LOC ?OBJ ?LOC"))
  (break)
  (let ((knife (desig:make-designator :object '((:type :knife) (:color :blue)))))
    (roslisp:ros-info colloquium "completing designator ~a" knife)
    (cram-robot-memory:complete-with-next-on-failure
     #'(lambda(designator)
         (roslisp:ros-info colloquium "trying to get the following object in hand: ~C~a"
                           #\linefeed designator)
         (break)
         (plan-lib:achieve `(plan-lib:object-in-hand ,designator)))
     knife
     "ACHIEVE"
     "OBJECT-IN-HAND ?OBJ"
     "?OBJ")
    (roslisp:ros-info colloquium "finished!")))
