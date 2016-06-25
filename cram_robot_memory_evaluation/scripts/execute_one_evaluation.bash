#!/bin/bash

SCRIPT_FILE=`readlink -f $0`
SCRIPT_DIRECTORY=`dirname $SCRIPT_FILE`
cd $SCRIPT_DIRECTORY/..

rm -rf kitchens
mkdir kitchens
$SCRIPT_DIRECTORY/sample_kitchens.py --locations=data/locations.csv --objtypes=data/objects.csv --outdir=kitchens $@
cd kitchens

for KITCHEN in kitchen*; do
	$SCRIPT_DIRECTORY/handle_objects_in_kitchen.lisp $KITCHEN
done
