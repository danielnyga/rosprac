This package contains the code for the evaluation conducted in the master's thesis of Marc Niehaus.
It generates 25 "kitchens" containing diverse objects which have to be transported to the pancake_table.
Make sure that a MongoDB is running and that the MongoDB C driver (see the owl_memory_converter readme for details) is installed before executing the demo.
The evaluation can be executed by running 

rosrun cram_robot_memory_evaluation execute_evaluation.bash

Unfortunately, there are some deadlocks in KnowRob and/or CRAM and thus, the evaluation will probably not complete in the first run.
Therefore, it is possible to restart it by executing the command 

rosrun cram_robot_memory_evaluation execute_evaluation.bash <phase> <kitchen_number>

where <phase> is either training or test and <kitchen-number> is 1-20 in the test phase and 1-5 in the training phase.
