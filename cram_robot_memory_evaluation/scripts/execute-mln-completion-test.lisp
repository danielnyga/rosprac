#!/usr/bin/sbcl --script
(load #P"/opt/ros/indigo/share/roslisp/scripts/roslisp-sbcl-init")
(ros-load:load-system "cram_robot_memory_evaluation"  :cram-robot-memory-evaluation)
(if (not (= (length *posix-argv*) 2))
    (print "Usage: execute-mln-completion-test <kitchen>")
    (cram-robot-memory-evaluation:execute-mln-completion-test
     (nth 1 *posix-argv*)))
