#!/bin/bash
export PICO_SDK_PATH=/home/alex/software/pico-sdk
cd build
cmake ..
make -j4
cd ..
