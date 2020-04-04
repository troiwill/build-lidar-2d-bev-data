#!/bin/bash

echo "Launching CMAKE."
cmake .. -DCMAKE_INSTALL_PREFIX=../bin

echo "Building project."
make -j && make install
