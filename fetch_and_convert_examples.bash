#!/bin/bash

# Problematic log files:
# ======================
# Pick-and-Place/pr2-general-pick-and-place_0/episode2/log.owl	No logged designators
# Pick-and-Place/xsens-kitchen-pnp_0/episode1/log.owl		Invalid log file (TODO: fix it...)
# Pizza-Making/forth-pizza_0/episode1/log.owl			Invalid log file (TODO: fix it...)
# Pancake-Making/sim-pancake_0/episode2/log.owl			No designator collection in database
# Pancake-Making/sim-pancake_0/episode6/log.owl			No designator collection in database
# Pancake-Making/sim-pancake_0/episode10/log.owl		No designator collection in database
# Pancake-Making/sim-pancake_0/episode9/log.owl			No designator collection in database
# Pancake-Making/sim-pancake_0/episode7/log.owl			No designator collection in database
# Pancake-Making/sim-pancake_0/episode18/log.owl		No designator collection in database
# Pancake-Making/sim-pancake_0/episode4/log.owl			No designator collection in database
# Pancake-Making/sim-pancake_0/episode14/log.owl		No designator collection in database
# Pancake-Making/sim-pancake_0/episode5/log.owl			No designator collection in database
# Pancake-Making/sim-pancake_0/episode15/log.owl		No designator collection in database
# Pancake-Making/sim-pancake_0/episode19/log.owl		No designator collection in database
# Pancake-Making/sim-pancake_0/episode1/log.owl			No designator collection in database
# Pancake-Making/sim-pancake_0/episode13/log.owl		No designator collection in database
# Pancake-Making/sim-pancake_0/episode3/log.owl			No designator collection in database
# Pancake-Making/sim-pancake_0/episode20/log.owl		No designator collection in database
# Pancake-Making/sim-pancake_0/episode12/log.owl		No designator collection in database
# Pancake-Making/sim-pancake_0/episode17/log.owl		No designator collection in database
# Pancake-Making/sim-pancake_0/episode11/log.owl		No designator collection in database
# Pancake-Making/sim-pancake_0/episode8/log.owl			No designator collection in database
# Pancake-Making/sim-pancake_0/episode16/log.owl		No designator collection in database


LOGFILES=(
	"Pizza-Making/boxy-rolling_0/episode2/log.owl"
	"Pizza-Making/boxy-rolling_0/episode1/log.owl"
	"Pizza-Making/sim-pizza_0/episode1/log.owl"
	"Pick-and-Place/pr2-prediction_0/episode1/log.owl"
	"Pick-and-Place/pr2-general-pick-and-place_0/episode1/log.owl"
	"Pick-and-Place/pr2-kitchen-tablesetting_0/episode1/log.owl"
	"Pick-and-Place/pr2-pizza_0/episode1/log.owl"
	"Safe-Interaction/boxy-safe_0/episode2/log.owl"
	"Safe-Interaction/boxy-safe_0/episode1/log.owl"
	"Safe-Interaction/boxy-safe_0/episode3/log.owl"
	"Chemical-Laboratory/pr2-chemlab_0/episode1/log.owl"
	"Perception/table-top-scene1/episode1/log.owl"
)

cd pracmln
python setup.py
source env.sh
cd ..
mkdir -p logs
for LOGFILE in ${LOGFILES[*]}; do
	if [ ! -f "logs/$LOGFILE" ]; then
		wget --mirror --no-host-directories -P logs ftp://open-ease-stor.informatik.uni-bremen.de/$LOGFILE
	fi
done

mkdir -p learnt_mlns
cd learnt_mlns
script -qc "rosrun robot_memory service debug" >> service.log 2>&1 &
sleep 3
for LOGFILE in ${LOGFILES[*]}; do
	echo $LOGFILE
	echo "=======================" >> client.log
	echo "    " $LOGFILE >> client.log
	echo "=======================" >> client.log
	rosrun owl_memory_converter converter/build/install/converter/bin/converter "../logs/$LOGFILE" >> client.log 2>&1
done
echo "Finished!"
kill $!
rm typescript
