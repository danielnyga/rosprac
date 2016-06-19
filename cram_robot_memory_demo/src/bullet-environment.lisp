(in-package :cram-robot-memory-demo)

;the code in this file is mainly copied from the spatial relations demo

(defmethod clean-and-spawn-kitchen-and-robot((environment (eql :bullet)))
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

(defmethod spawn-object((environment (eql :bullet)) object-name object-type mass pose)
  (cram-prolog:prolog `(and (cram-bullet-reasoning:bullet-world ?w)
                            (assert (cram-bullet-reasoning:object ?w
                                             :mesh ,object-name ,pose
                                             :mass ,mass :color (1 0 0) :mesh ,object-type)))))

(defmethod execute-in-environment((environment (eql :bullet)) function)
  (cram-projection:with-projection-environment
      projection-process-modules::pr2-bullet-projection-environment
      (funcall function)))

(cram-prolog:def-fact-group costmap-metadata ()
  (cram-prolog:<- (location-costmap:costmap-size 12 12))
  (cram-prolog:<- (location-costmap:costmap-origin -6 -6))
  (cram-prolog:<- (location-costmap:costmap-resolution 0.05))
  (cram-prolog:<- (location-costmap:costmap-padding 0.2))
  (cram-prolog:<- (location-costmap:costmap-manipulation-padding 0.2))
  (cram-prolog:<- (location-costmap:costmap-in-reach-distance 0.9))
  (cram-prolog:<- (location-costmap:costmap-reach-minimal-distance 0.1)))

(cram-prolog:def-fact-group semantic-map-data ()
  (cram-prolog:<- (semantic-map-object-name :kitchen)))
