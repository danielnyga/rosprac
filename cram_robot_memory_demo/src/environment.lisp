(in-package :cram-robot-memory-demo)

(defgeneric spawn-object(environment object-name object-type pose))

(defgeneric clean-and-spawn-kitchen-and-robot(environment))

(defgeneric execute-in-environment(environment function))
