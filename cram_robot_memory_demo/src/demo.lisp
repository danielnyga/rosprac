(in-package :cram-robot-memory-demo)

(defun execute-demo()
  (create-training-data)
  (execute-test))

(defun create-training-data()
  (roslisp-utilities::startup-ros)
  (cram-robot-memory:start-learning)
  (init-kitchen-and-pr2-in-bullet)
  (spawn-object-in-bullet :mug-1 :mug 0.2 '((-0.9 -0.6 0.8) (0 0 0 1)))
  (spawn-object-in-bullet :mug-2 :mug 0.2 '((1.5 0.9 0.9) (0 0 0 1)))
  (execute-in-simulation #'(lambda() (perceive-mug-at-location
                                      "CounterTop" "kitchen_sink_block_counter_top")))
  (execute-in-simulation #'(lambda() (perceive-mug-at-location
                                      "CounterTop" "kitchen_sink_block_counter_top")))
  (execute-in-simulation #'(lambda() (perceive-mug-at-location
                                      "Cupboard" "pancake_table")))
  (execute-in-simulation #'(lambda() (grasp-mug-at-pose '(-0.9 -0.6 0.8) '(0 0 0 1))))
  (cram-robot-memory:complete-learning nil)
  (roslisp-utilities::shutdown-ros))

(cram-language:def-top-level-cram-function grasp-mug-at-pose(position orientation)
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

(cram-language:def-top-level-cram-function perceive-mug-at-location(location-on location-name)
  (cram-language-designator-support:with-designators
      ((mug-location :location `((:on ,location-on) (:name ,location-name)))
       (mug :object `((:type :mug) (:at ,mug-location))))
    (cram-plan-library:perceive-object 'cram-plan-library:a mug)))

(cram-language:def-top-level-cram-function perceive-and-grasp-mug()
  (cram-robot-memory:complete-with-next-on-failure
   #'(lambda(mug-p)
       (cram-plan-library:perceive-object 'cram-plan-library:a mug-p)
       (cram-robot-memory:complete-with-next-on-failure
        #'(lambda(mug-g) (cram-plan-library:achieve `(cram-plan-library:object-in-hand ,mug-g)))
        mug-p 'achieve "OBJECT-IN-HAND ?OBJ"))
   (cram-designators:make-designator :object `((:type :mug)))
   'perceive-object))
          
(defun execute-test()
  (roslisp-utilities::startup-ros)
  (init-kitchen-and-pr2-in-bullet)
  (spawn-object-in-bullet :mug-3 :mug 0.2 '((-0.9 -0.7 0.8) (0 0 0 1)))
  (execute-in-simulation #'perceive-and-grasp-mug)
  (sleep 1); Wait for cram beliefstate to complete...TODO: Find anther solution...
  (roslisp-utilities::shutdown-ros))

(defun execute-in-simulation(function)
  (cram-projection:with-projection-environment
      projection-process-modules::pr2-bullet-projection-environment
      (funcall function)))

;this code is basically copied from the spatial relations demo
(defun init-kitchen-and-pr2-in-bullet()
	(let 
	  ((robot-urdf (cl-urdf:parse-urdf (roslisp:get-param "robot_description_lowres")))
	   (kitchen-urdf (cl-urdf:parse-urdf (roslisp:get-param "kitchen_description"))))
	  (cram-prolog:prolog `(and 
                          (cram-bullet-reasoning:clear-bullet-world)
                          (cram-bullet-reasoning:bullet-world ?w)
                          (cram-robot-interfaces:robot ?robot)
                          (cram-bullet-reasoning:debug-window ?w)
                          (assert (cram-bullet-reasoning:object ?w
                                             :static-plane floor ((0 0 0) (0 0 0 1))
                                             :normal (0 0 1) :constant 0))
                          (assert (cram-bullet-reasoning:object ?w
                                             :box wall ((2.3 0.5 1) (0 0 0 1))
                                             :mass 100 :size (1 4 2)))
                          (assert (cram-bullet-reasoning:object ?w
                                             :semantic-map kitchen ((-3.45 -4.35 0) (0 0 1 0))
                                             :urdf ,kitchen-urdf))
                          (assert (cram-bullet-reasoning:object ?w
                                             :urdf ?robot ((0 0 0) (0 0 0 1))
                                             :urdf ,robot-urdf))
                          (cram-robot-interfaces:robot-arms-parking-joint-states ?robot
                                                                                 ?joint-states)
                          (assert (cram-bullet-reasoning:joint-state ?w ?robot ?joint-states))
                          (assert (cram-bullet-reasoning:joint-state
                                   ?w ?robot (("torso_lift_joint" 0.16825d0))))))))

;this code is basically copied from the spatial relations demo
(cram-prolog:def-fact-group costmap-metadata ()
  (cram-prolog:<- (location-costmap:costmap-size 12 12))
  (cram-prolog:<- (location-costmap:costmap-origin -6 -6))
  (cram-prolog:<- (location-costmap:costmap-resolution 0.05))
  (cram-prolog:<- (location-costmap:costmap-padding 0.2))
  (cram-prolog:<- (location-costmap:costmap-manipulation-padding 0.2))
  (cram-prolog:<- (location-costmap:costmap-in-reach-distance 0.9))
  (cram-prolog:<- (location-costmap:costmap-reach-minimal-distance 0.1)))

;this code is basically copied from the spatial relations demo
(cram-prolog:def-fact-group semantic-map-data ()
  (cram-prolog:<- (semantic-map-object-name :kitchen)))

;this code is basically copied from the spatial relations demo
(defun spawn-object-in-bullet(object-name object-type mass pose)
  (cram-prolog:prolog `(and (cram-bullet-reasoning:bullet-world ?w)
                            (assert (cram-bullet-reasoning:object ?w
                                             :mesh ,object-name ,pose
                                             :mass ,mass :color (1 0 0) :mesh ,object-type)))))
