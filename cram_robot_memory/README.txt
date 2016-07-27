This package provides access to the "lifelong learning of first-order probabilistic models for everyday robot manipulation" from CRAM.  It provides functions for learning a MLN and accessing the MLN to complete a designator, to compute a success probability or to execute another query.
Make sure that a MongoDB is running and that the MongoDB C driver (see the owl_memory_converter readme for details) is installed before doing anything else.
To use the complete functionality, the following launch file has to be started:

roslaunch cram_robot_memory robot_memory.launch

This will also start the required nodes to caputre log files and to convert log files to MLNs.
Recording experiences can be started by the "start-learning" function. This function will automatically load cram_beliefstate functions.  When this is done, the "complete-learning" function can be called to actually learn a MLN. It triggers semrec to export a log file, sends this log file to the robot_memory service and makes the robot_memory service learn a mln.
The MLN can then be acccessed by the function "with-completed-designator", "complete-with-next-on-failure", "get-failure-probability" and "execute-general-query".
