#!/bin/bash

type=$1

if [ -d "./build" ]; then
	rm -rf ./build
	mkdir ./build/
fi

cd ./build/

if [ "$type" == "-r" ] ; then
	echo "Generating project for release"
	cmake -DCMAKE_BUILD_TYPE=Release ../src/
else
	echo "Generating project for debug"
	cmake -DCMAKE_BUILD_TYPE=Debug ../src/
fi

cd ..
./make.sh
