#!/bin/bash

core_num=$(grep -c ^processor /proc/cpuinfo)

binfilename=sos_studio
binpath=./deployqt/bin

# run
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"${PWD}/deployqt/lib"
#echo "$LD_LIBRARY_PATH"
#gdb --args $binpath/$binfilename $1
$binpath/$binfilename $1
