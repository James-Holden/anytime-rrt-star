#!/bin/bash
./planner/planner < $1 > latest.txt
./visualizer/visualizer < latest.txt