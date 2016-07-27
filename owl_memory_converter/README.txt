This package is designated to combine the OWL log files and the information from a MongoDB in a single task tree representation.
To make the package work properly, the MongoDB C API must be installed on the computer (It is not referenced as dependency!). Otherwise, the designators will not be available.
The package actually provides two functionalities.
The first one is a task tree visualization. It can be started with

rosrun owl_memory_converter taskHierarchy/build/install/taskHierarchy/bin/taskHierarchy logfile

Note that the log file path passed as last argument is also used to determine the name of the MongoDB. Therefore, the directory layout must be the same as for the experiments available at http://www.knowrob.org/doc/docker.
The second functionality is to send log files to the robot_memory service. To achieve that, the following command has to be executed:

rosrun owl_memory_converter converter/build/install/converter/bin/converter logfile

In this case, there are the same restrictions for the log file path as for the task tree visualization. The execution of the command shown above automatically reads the log file, transfers it to the robot_memory service and waits until the MLNs have been learned. However, there is also the possibility to start the converter as service. To achieve this, the following command can be executed:

rosrun owl_memory_converter converter/build/install/converter/bin/converter

Then, the owl_memory_converter will provide a Conversion service such that the service call will contain the log file name.

This package uses the OWL API (http://owlapi.sourceforge.net/) to read the log file.
