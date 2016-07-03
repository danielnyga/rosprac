#!/bin/bash

SCRIPT_FILE=`readlink -f $0`
SCRIPT_DIRECTORY=`dirname $SCRIPT_FILE`
cd $SCRIPT_DIRECTORY/..

rm -rf kitchens
mkdir kitchens
$SCRIPT_DIRECTORY/sample_kitchens.py --locations=data/locations.csv --objtypes=data/objects.csv --outdir=kitchens $@

EXTEND_MLNS="false"
for KITCHEN in kitchens/training-kitchen*; do
	roslaunch cram_robot_memory_demo demo.launch&
	ROSLAUNCH_PID=$!
	sleep 30
	echo "=================================================="
	echo "operating on $KITCHEN"
	echo "=================================================="
	sbcl --dynamic-space-size 4096 --script $SCRIPT_DIRECTORY/handle_objects_in_kitchen.lisp $KITCHEN train $EXTEND_MLNS
	kill $ROSLAUNCH_PID
	sleep 10
	EXTEND_MLNS="true"
done