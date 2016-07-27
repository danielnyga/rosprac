This package provides a service able learn or update a MLN from a task tree. This service can be started by executing:

rosrun robot_memory service

Then, the task tree has to be sent to the running ROS node. Therefore, one has to state that the messages will be sent. This is done by calling the Trigger service /robot_memory/start_collecting_training_data. Afterwards, the task tree can be published to the /robot_memory/state topic in form of RobotState messages from the task_tree_msgs package. Finally, the LearningTrigger service /robot_memory/learn has to be called. This causes MLNs to be learned and placed in ../learned_mlns. If the MLNs already exist, they are updated. 
Actually, two MLNs are generated: In contrast to the task_and_designator.mln MLN, the designator.mln does not contain task information and is thus more precise and more performant if no task information is needed in query or evidence.

There are a few more options that can be passed to the service executable:

rosrun robot_memory service --debug --include-perform

The --debug option prints the task tree to the command line before learning. Moreover, a database file is created. (Unfortunately, the databases have the wrong form - actually, more databases with less ground atoms have to be created. The MLN weights are calculated correctly, though).
The --include-perform option also includes action designators used as arguments for the process modules in the MLNs.
