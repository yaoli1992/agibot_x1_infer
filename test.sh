#!/bin/bash

# exit on error and print each command
set -ex

# cmake
cmake -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=./build/install \
    -DXYBER_X1_INFER_BUILD_TESTS=ON \
    -DXYBER_X1_INFER_SIMULATION=ON \
    $@

if [ -d ./build/install ]; then
    rm -rf ./build/install
fi

cmake --build build --config Release --target install --parallel $(nproc)
