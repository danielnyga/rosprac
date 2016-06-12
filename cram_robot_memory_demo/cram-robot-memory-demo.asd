(asdf:defsystem cram-robot-memory-demo
  :depends-on (roslisp
               cram-rosmln
               cram-designators
               cram-language
               cl-transforms-stamped
               cl-transforms
	       cram-spatial-relations-costmap
               cl-urdf
               cram-location-costmap
               cram-prolog
               roslisp-utilities
               cram-semantic-map-costmap
               cram-semantic-map-utils
               cram-robot-pose-gaussian-costmap
               cram-bullet-reasoning
               cram-pr2-description
               cram-bullet-reasoning-belief-state
               cram-pr2-synch-projection-pms
               cram-occupancy-grid-costmap
               cram-plan-library
               cram-bullet-reasoning-designators
               cram-pr2-designators
               cram-semantic-map-designators
               alexandria
               cram-bullet-reasoning-utilities
	       )
  :components
  ((:module "src"
    :components
    ((:file "package")
     (:file "demo" :depends-on ("package"))))))
