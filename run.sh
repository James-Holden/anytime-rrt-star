#!/bin/bash
./planner/planner $2 $3 $4 $5 $6 < $1 > latest.txt
./visualizer/visualizer < latest.txt