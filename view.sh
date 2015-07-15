#!/bin/bash

cd ./output/

k=0
FILES=./*_addedPoint_*
for f in $FILES
do
	echo $f

	# Open geomview and catch its pid
	geomview $f &
	last_pid=$!
	
	# Select the interesting window and take the screenshot
	sleep 0.8
	wmctrl -a "Camera (Euclidean view)"
	sleep 0.2
	gnome-screenshot -w --file=/home/rodrigo/Pictures/$k.png
	k=$((k+1))

	kill -9 $last_pid
done
