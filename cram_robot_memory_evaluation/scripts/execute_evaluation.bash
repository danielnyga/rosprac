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

if ([ $# -ne 2 ] && [ $# -ne 0 ]) || ([ $# -eq 2 ] && [ "$1" != "test" ] && [ "$1" != "training" ]) ; then
	echo -e "Usage: execute_evaluation.bash [<phase> <kitchen_number>]\n       with <phase> beeing test or training and <kitchen_number> the number to start with"
	exit 1
fi
PHASE="training"
KITCHEN_NUMBER=0
if [ $# -eq 2 ]; then
	PHASE=$1
	KITCHEN_NUMBER=$2
fi

SCRIPT_FILE=`readlink -f $0`
SCRIPT_DIRECTORY=`dirname $SCRIPT_FILE`
cd $SCRIPT_DIRECTORY/..

if [ $PHASE == "training" ]; then
	if [ $KITCHEN_NUMBER -eq 0 ]; then
	rm -rf kitchens
		mkdir kitchens
#		$SCRIPT_DIRECTORY/sample_kitchens.py --locations=data/locations.csv --objtypes=data/objects.csv --outdir=kitchens --dpmin 1 --dpmax 2 --dkpop 5 --dvpop 5 --train-kitchens 15 --test-kitchens 5 --objects 10
		$SCRIPT_DIRECTORY/sample_kitchens.py --locations=data/locations.csv --objtypes=data/objects.csv --outdir=kitchens --dpmin 1 --dpmax 2 --dkpop 5 --dvpop 5 --train-kitchens 1 --test-kitchens 1 --objects 2
	fi
	EXTEND_MLNS="false"
	for KITCHEN in kitchens/training-kitchen-*; do
		CURRENT_KITCHEN_NUMBER=`echo $KITCHEN | cut -d"-" -f 3 | cut -d"." -f 1`
		if [ $CURRENT_KITCHEN_NUMBER -ge $KITCHEN_NUMBER ]; then
			start_ros_environment
			echo "=================================================="
			echo "training on $KITCHEN"
			echo "=================================================="
			$LISP_INTERPRETER $SCRIPT_DIRECTORY/grasp-objects-for-training.lisp $KITCHEN $EXTEND_MLNS
			EXTEND_MLNS="true"
			kill_ros_environment
		fi
	done
fi

for KITCHEN in kitchens/test-kitchen-*; do
	CURRENT_KITCHEN_NUMBER=`echo $KITCHEN | cut -d"-" -f 3 | cut -d"." -f 1`
	if [ $CURRENT_KITCHEN_NUMBER -ge $KITCHEN_NUMBER ]; then
		start_ros_environment
		echo "=================================================="
		echo "theoretical completion test on $KITCHEN"
		echo "=================================================="
		$LISP_INTERPRETER $SCRIPT_DIRECTORY/execute-theoretical-test.lisp $KITCHEN data/locations.csv
		kill_ros_environment
		start_ros_environment
		echo "=================================================="
		echo "informed test on $KITCHEN"
		echo "=================================================="
		$LISP_INTERPRETER $SCRIPT_DIRECTORY/execute-informed-test.lisp $KITCHEN
		kill_ros_environment
		start_ros_environment
		echo "=================================================="
		echo "mln completion test on $KITCHEN"
		echo "=================================================="
		$LISP_INTERPRETER $SCRIPT_DIRECTORY/execute-mln-completion-test.lisp $KITCHEN
		kill_ros_environment
		start_ros_environment
		echo "=================================================="
		echo "naive completion test on $KITCHEN"
		echo "=================================================="
		$LISP_INTERPRETER $SCRIPT_DIRECTORY/execute-naive-completion-test.lisp $KITCHEN data/locations.csv
		kill_ros_environment
	fi
done

$SCRIPT_DIRECTORY/calculate_statistics.py kitchens
