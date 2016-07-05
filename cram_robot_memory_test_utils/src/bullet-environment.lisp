(in-package :cram-robot-memory-test-utils)

;the code in this file is mainly copied from the spatial relations demo or from gayas mail

(defmethod clean-and-spawn-kitchen-and-robot((environment (eql :bullet)))
	(let 
	  ((kitchen-urdf (open-cupboards (cl-urdf:parse-urdf
                                    (roslisp:get-param "kitchen_description"))))
     (robot-urdf (cl-urdf:parse-urdf (roslisp:get-param "robot_description_lowres"))))
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

(defmethod spawn-object((environment (eql :bullet)) object-name object-type pose)
  (let ((name (symbol-value (intern object-name "KEYWORD"))))
    (bullet-reasoning-utilities:spawn-object name object-type :color '(0 0 1) :pose pose)
    (cram-bullet-reasoning:simulate cram-bullet-reasoning:*current-bullet-world* 5)))

(defmethod execute-in-environment((environment (eql :bullet)) function)
  (cram-projection:with-projection-environment
      projection-process-modules::pr2-bullet-projection-environment
      (cram-language:top-level (funcall function))))

(defun open-cupboards(kitchen-urdf)
  (let*((urdf-01 (open-door kitchen-urdf "fridge_block_fridge_joint"))
        (urdf-02 (open-drawer urdf-01 "oven_block_drawer_oven_center_joint"))
        (urdf-03 (open-drawer urdf-02 "island_block_drawer_island_col1_center_joint"))
        (urdf-04 (open-drawer urdf-03 "island_block_drawer_island_col3_center_joint"))
        (urdf-05 (open-drawer urdf-04 "sink_block_drawer_sink_col1_center_joint"))
        (urdf-06 (remove-door urdf-05 "drawer_oven_oven_link"))
        (urdf-07 (remove-door urdf-06 "drawer_sink_col2_link")))    
    urdf-07))

(defun open-door(kitchen-urdf joint)
  (setf (slot-value (gethash joint (cl-urdf:joints kitchen-urdf))'cl-urdf:origin)
        (cl-transforms:make-transform
         (cl-transforms:make-identity-vector)
         (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 0 0 1) 2.0)))
  kitchen-urdf)

(defun open-drawer(kitchen-urdf joint)
  (setf (slot-value (gethash joint (cl-urdf:joints kitchen-urdf)) 'cl-urdf:origin)
        (cl-transforms:make-transform
         (cl-transforms:make-3d-vector 0.4 0 0)
         (cl-transforms:make-identity-rotation)))
  kitchen-urdf)

(defun remove-door(kitchen-urdf door-name)
  (setf (slot-value (gethash (concatenate 'string "handle_" door-name)
                             (slot-value kitchen-urdf 'cl-urdf:links))
                    'cl-urdf:collision)
        nil)
  (setf (slot-value (gethash (concatenate 'string door-name)
                             (slot-value kitchen-urdf 'cl-urdf:links))
                    'cl-urdf:collision)
        nil)
  kitchen-urdf)
  

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
