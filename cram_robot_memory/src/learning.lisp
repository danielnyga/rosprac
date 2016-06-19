(in-package :cram-robot-memory)

(define-condition learning-failed (simple-condition) ())

(defun start-learning()
  ;avoid static dependencies to cram_beliefstate when learning is not necessary...
  (ros-load:load-system "cram_beliefstate" :cram-beliefstate)
)

(defun complete-learning(extend-old-models)
  (funcall (find-symbol "EXTRACT-OWL-FILE" "BELIEFSTATE") "log.owl")
  (funcall (find-symbol "ENABLE-LOGGING" "BELIEFSTATE") nil)
  (let ((convert-service-name "owl_memory_converter/convert"))
    (cond  ((not (wait-for-service convert-service-name 30))
            (roslisp:ros-error (cram-rosmln) "timeout while waiting for service ~a!"
                               convert-service-name)
            (cram-language:fail 'learning-failed))
           ((not (owl_memory_converter-srv:success
                  (roslisp:call-service convert-service-name
                                        'owl_memory_converter-srv:Conversion
					:extend_old_model extend-old-models
                                        :owl_file_name
                                        "../logs/robot_memory/exp-0/log.owl")))
            (cram-language:fail 'learning-failed))
           (t t))))
