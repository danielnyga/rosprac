#!/bin/bash

LISP_INTERPRETER="sbcl --dynamic-space-size 4096 --script"

function start_ros_environment {
	roslaunch cram_robot_memory_demo demo.launch&
	ROSLAUNCH_PID=$!
	sleep 30
}

function kill_ros_environment {
	kill $ROSLAUNCH_PID
	sleep 10
}

SCRIPT_FILE=`readlink -f $0`
SCRIPT_DIRECTORY=`dirname $SCRIPT_FILE`
cd $SCRIPT_DIRECTORY/..

rm -rf kitchens
mkdir kitchens
$SCRIPT_DIRECTORY/sample_kitchens.py --locations=data/locations.csv --objtypes=data/objects.csv --outdir=kitchens --dpmin 1 --dpmax 2 --dkpop 5 --dvpop 5 --train-kitchens 15 --test-kitchens 5 --objects 10

EXTEND_MLNS="false"
for KITCHEN in kitchens/training-kitchen*; do
	start_ros_environment
	echo "=================================================="
	echo "training on $KITCHEN"
	echo "=================================================="
	$LISP_INTERPRETER $SCRIPT_DIRECTORY/grasp-objects-for-training.lisp $KITCHEN $EXTEND_MLNS
	EXTEND_MLNS="true"
	kill_ros_environment
done


for KITCHEN in kitchens/test-kitchen*; do
	start_ros_environment
	echo "=================================================="
	echo "completion test on $KITCHEN"
	echo "=================================================="
	$LISP_INTERPRETER $SCRIPT_DIRECTORY/grasp-objects-for-completion-test.lisp $KITCHEN
	echo "=================================================="
	echo "comparison test on $KITCHEN"
	echo "=================================================="
	$LISP_INTERPRETER $SCRIPT_DIRECTORY/grasp-objects-for-comparison-test.lisp $KITCHEN data/locations.csv
	kill_ros_environment
done

$SCRIPT_DIRECTORY/calculate_statistics.py kitchens/completion.csv kitchens/comparison.csv
