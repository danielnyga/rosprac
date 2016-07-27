This package includes some demos demonstrating the abilities of the cram_robot_memory package and thus of the whole lifelong learning system.
Make sure that a MongoDB is running and that the MongoDB C driver (see the owl_memory_converter readme for details) is installed before doing anything else.
To execute the demos, all necessary ros nodes have to be started. Therefore, the following command has to be executed:

roslaunch cram_robot_memory_demo demo.launch

Then, in roslisp_repl, one has to execute the following commands:

(ros-load:load-system "cram_robot_memory_demo" :cram-robot-memory-demo)
(cram-robot-memory-demo:execute-perceive-and-grasp-mug-demo)

or 

(ros-load:load-system "cram_robot_memory_demo" :cram-robot-memory-demo)
(cram-robot-memory-demo:execute-success-estimation-demo)

In the "perceive and grasp mug demo", the PR2 is told to perceive a mug twice at the kitchen_sink_block_counter_top and once at the pancake_table during in a training phase.  Moreover, it is told to grasp an object given a handle in the training phase.  Afterwards, in the test phase, it infers the most likely completion for perceiving the mug and tries to find it until it has been found on the pancake_table.  Since the PR2 first looks at the kitchen_sink_block, the completion was successful. Then, it infers the completion for grasping based on that designator and grasps the mug.
In the "success estimation" demo, the PR2 is told to grasp a fork and a mug at different locations during a training phase. Some of the grasp attempts will fail.  Afterwards, the success probability will be inferred in the test phase and compared with the expected probability.
