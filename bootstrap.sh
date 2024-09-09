#!/bin/bash

mkdir -p build

pushd build
# build for the devkit
# cmake -C ../devkit_config_modified.cmake -G Ninja -D CMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=YES  ..
# build for the devkit factory test demo - The Boot image will never exit
# cmake -C ../devkit_factorytest_config.cmake -G Ninja -D CMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=YES  ..
# build for the real hw factory test - The Boot image will never exit
# cmake -C ../realhw_factorytest_config.cmake -G Ninja -D CMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=YES  ..
# build for the real hw v2
cmake -C ../real_hw_v2.cmake -G Ninja -D CMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=YES  ..
popd