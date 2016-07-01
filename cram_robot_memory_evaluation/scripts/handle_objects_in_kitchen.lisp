#!/usr/bin/sbcl --script
(load #P"/opt/ros/indigo/share/roslisp/scripts/roslisp-sbcl-init")
(ros-load:load-system "cram_robot_memory_evaluation"  :cram-robot-memory-evaluation)
(if (/= (length *posix-argv*) 3)
    (print "Usage: handle_objects_in_kitchen <kitchen> <extend existing>")
    (cram-robot-memory-evaluation:handle-objects-in-kitchen
     (nth 1 *posix-argv*)
     (cond ((equal (nth 2 *posix-argv*) "true") t)
           ((equal (nth 2 *posix-argv*) "false") nil)
           (t (print "<extend existing> must be either true or false!")))))
