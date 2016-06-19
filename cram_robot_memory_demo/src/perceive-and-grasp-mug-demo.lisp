(in-package :cram-robot-memory-demo)

(defun execute-perceive-and-grasp-mug-demo-in-bullet()
  (execute-perceive-and-grasp-mug-demo :bullet))

(defun execute-perceive-and-grasp-mug-demo(environment)
  (execute-training-and-test-functions environment
                                       #'create-perceive-and-grasp-mug-training-data
                                       #'test-perceive-and-grasp-mug))

(defun create-perceive-and-grasp-mug-training-data(environment)
  (spawn-object environment :mug-1 :mug 0.2 '((-0.9 -0.6 0.8) (0 0 0 1)))
  (spawn-object environment :mug-2 :mug 0.2 '((1.5 0.9 0.9) (0 0 0 1)))
  (perceive-mug-at-location "CounterTop" "kitchen_sink_block_counter_top")
  (perceive-mug-at-location "CounterTop" "kitchen_sink_block_counter_top")
  (perceive-mug-at-location "Cupboard" "pancake_table")
  (grasp-mug-at-pose '(-0.9 -0.6 0.8) '(0 0 0 1)))

(defun test-perceive-and-grasp-mug(environment)
  (spawn-object environment :mug-3 :mug 0.2 '((-0.9 -0.6 0.8) (0 0 0 1)))
  (perceive-and-grasp-mug))

(cram-language:def-cram-function grasp-mug-at-pose(position orientation)
  (cram-language-designator-support:with-designators
      ((mug-loc :location `((:pose ,(cl-transforms-stamped:make-pose-stamped 
                                     "map"
                                     0.0
                                     (cl-transforms:make-3d-vector
                                      (nth 0 position) (nth 1 position) (nth 2 position))
                                     (cl-transforms:make-quaternion
                                      (nth 0 orientation) (nth 1 orientation)
                                      (nth 2 orientation) (nth 3 orientation))))))
       (handle-loc :location `((:pose ,(cl-transforms:make-pose
                                        (cl-transforms:make-3d-vector -0.005 0.0 0.0)
                                        (cl-transforms:make-quaternion 0.5 -0.5 -0.5 0.5)))))
       (handle :object `((:type :handle) (:at ,handle-loc)))
       (mug :object `((:type :mug) (:handle ,handle) (:at ,mug-loc))))
    (cram-plan-library:achieve `(cram-plan-library:object-in-hand ,mug))))

(cram-language:def-cram-function perceive-mug-at-location(location-on location-name)
  (cram-language-designator-support:with-designators
      ((mug-location :location `((:on ,location-on) (:name ,location-name)))
       (mug :object `((:type :mug) (:at ,mug-location))))
    (cram-plan-library:perceive-object 'cram-plan-library:a mug)))

(cram-language:def-cram-function perceive-and-grasp-mug()
  (cram-robot-memory:complete-with-next-on-failure
   #'(lambda(mug-p)
       (cram-plan-library:perceive-object 'cram-plan-library:a mug-p)
       (cram-robot-memory:complete-with-next-on-failure
        #'(lambda(mug-g) (cram-plan-library:achieve `(cram-plan-library:object-in-hand ,mug-g)))
        mug-p 'achieve "OBJECT-IN-HAND ?OBJ"))
   (cram-designators:make-designator :object `((:type :mug)))
   'perceive-object))
