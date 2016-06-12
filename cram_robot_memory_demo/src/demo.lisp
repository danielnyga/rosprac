(in-package :cram-robot-memory-demo)

(defun execute-demo()
  (roslisp-utilities::startup-ros)
  (init-kitchen-and-pr2-in-bullet)
  (spawn-objects-in-bullet)
  (roslisp-utilities::shutdown-ros)
)

(defun init-kitchen-and-pr2-in-bullet()
  ;this code is basically copied from the spatial relations demo
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

(defun spawn-objects-in-bullet()
  (spawn-object-in-bullet "my-mug" :mug 0.2 '((1.3 0.4 0.9) (0 0 0 1)))
)

(defun spawn-object-in-bullet(object-name object-type mass pose)
  (cram-prolog:prolog `(and (cram-bullet-reasoning:bullet-world ?w)
                            (assert (cram-bullet-reasoning:object ?w
                                             :mesh ,object-name ,pose
                                             :mass ,mass :color (1 0 0) :mesh ,object-type)))))
