#!/bin/sh

if [ ! -d "./build" ]; then
  mkdir ./build/
fi

rm -rf ./build/*
cd ./build
cmake -DCMAKE_BUILD_TYPE=Release ../src/
cd ..
