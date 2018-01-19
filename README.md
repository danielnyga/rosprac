ROS bindings for the PRAC Natural-language interpreter
======================================================


To runthe server:

    $ catkin_make
    $ export PYTHONPATH=/path/to/prac/python2:$PYTHONPATH
    $ rosrun rosprac pracserver.py

To run the client:

    $ rosrun rosprac pracclient.py "Slice the pizza."
