#!/usr/bin/sbcl --script
(load #P"/opt/ros/indigo/share/roslisp/scripts/roslisp-sbcl-init")
(ros-load:load-system "cram_robot_memory_evaluation"  :cram-robot-memory-evaluation)
(if (not (= (length *posix-argv*) 3))
    (print "Usage: grasp-objects-for-training <kitchen> <extend existing>")
    (cram-robot-memory-evaluation:execute-training
     (nth 1 *posix-argv*)
     (cond ((equal (nth 2 *posix-argv*) "true") t)
           ((equal (nth 2 *posix-argv*) "false") nil)
           (t (print "<extend existing> must be either true or false!")))))
