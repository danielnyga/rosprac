(in-package :cram-robot-memory-test-utils)

(defgeneric spawn-object(environment object-name object-type pose)
  (:documentation
   "spawns an object object-type with the name object-name at the
    pose pose in the simulation environment environment.
    environment is usually :bullet
    object-name is a string uniquely identifying the object
    object-type is an object type, e.g. :mug.
    The available object types are dependent on the envornment.
    The pose is a list, e.g. ((1,2,3) (1,0,0,0)) where the first part
    is the position and the second one the orientation as quaternion."))

(defgeneric clean-and-spawn-kitchen-and-robot(environment)
  (:documentation
   "removes all objects including the robot from the simulation environment,
    spawns the IAI kitchen and a PR2 robot.
    Environment is usually :bullet"))

(defgeneric execute-in-environment(environment function)
  (:documentation
   "executes function, which is a cram plan, in the environment.
    environment is usually :bullet.
    function is a CRAM plan with no arguments,
    it does not have to be a top level cram plan."))
