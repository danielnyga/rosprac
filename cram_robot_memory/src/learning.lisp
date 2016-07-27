(in-package :cram-robot-memory)

(define-condition learning-failed (simple-condition) ())

(defun start-learning()
  "Starts recording function calls which should later be part of a MLN.
   This function essentially loads cram_beliefstate...."
  ;avoid static dependencies to cram_beliefstate when learning is not necessary...
  (ros-load:load-system "cram_beliefstate" :cram-beliefstate)
  (funcall (find-symbol "ENABLE-LOGGING" "BELIEFSTATE") t)
)

(defun complete-learning(extend-old-models)
  "Learns a MLN from the recorded function calls.
   This function requries robot_memory.launch to be started.
   extend-old-models has to be t if an existing MLN shall be updated and nil otherwise." 
  (let ((convert-service-name "owl_memory_converter/convert")
        (owl-file-path (concatenate
                        'string
                        (directory-namestring
                         (ros-load:ros-package-path "cram_robot_memory"))
                        "/logs/robot_memory/exp-0/log.owl")))
    (if (probe-file owl-file-path) (delete-file owl-file-path))
    (funcall (find-symbol "EXTRACT-OWL-FILE" "BELIEFSTATE") "log.owl")
    (funcall (find-symbol "ENABLE-LOGGING" "BELIEFSTATE") nil)   
    (cond  ((not (wait-for-service convert-service-name 30))
            (roslisp:ros-error (cram-rosmln) "timeout while waiting for service ~a!"
                               convert-service-name)
            (cram-language:fail 'learning-failed))
           ((not (owl_memory_converter-srv:success
                  (roslisp:call-service convert-service-name
                                        'owl_memory_converter-srv:Conversion
                                        :extend_old_model extend-old-models
                                        :owl_file_name
                                        owl-file-path)))
            (cram-language:fail 'learning-failed))
           (t t))))
