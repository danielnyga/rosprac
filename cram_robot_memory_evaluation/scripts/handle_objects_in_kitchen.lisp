#!/usr/bin/sbcl --script
(load #P"/opt/ros/indigo/share/roslisp/scripts/roslisp-sbcl-init")
(ros-load:load-system "cram_robot_memory_evaluation"  :cram-robot-memory-evaluation)
(if (/= (length *posix-argv*) 2)
    (print "Usage: handle_objects_in_kitchen <kitchen>")
    (cram-robot-memory-evaluation:handle-objects-in-kitchen (nth 1 *posix-argv*)))
