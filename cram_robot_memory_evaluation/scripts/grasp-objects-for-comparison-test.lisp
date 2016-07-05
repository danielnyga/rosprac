#!/usr/bin/sbcl --script
(load #P"/opt/ros/indigo/share/roslisp/scripts/roslisp-sbcl-init")
(ros-load:load-system "cram_robot_memory_evaluation"  :cram-robot-memory-evaluation)
(if (not (= (length *posix-argv*) 3))
    (print "Usage: grasp-objects-for-training <kitchen> <location file>")
    (cram-robot-memory-evaluation:execute-comparison-test
     (nth 1 *posix-argv*)
     (nth 2 *posix-argv*)))
