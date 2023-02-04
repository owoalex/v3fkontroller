#!/bin/bash
export PICO_SDK_PATH=/home/alex/software/pico-sdk
rm -r build
mkdir build
cd build
cmake ..
make -j4
cd ..
