#!/bin/bash

SCRIPT_FILE=`readlink -f $0`
SCRIPT_DIRECTORY=`dirname $SCRIPT_FILE`

$SCRIPT_DIRECTORY/execute_one_evaluation.bash --dpmin 0 --dpmax 1 --dkpop 10 --dvpop 10 --kitchens 10 --objects 7
