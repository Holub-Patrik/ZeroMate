#!/bin/bash

mkdir -p build >/dev/null 2>&1
cd build

cmake -G "Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE="../toolchain-arm-none-eabi-rpi0.cmake" ..

make
#make VERBOSE=1

cd ..
cp ./build/kernel_remote output/kernel_remote.elf
cp ./build/kernel_local output/kernel_local.elf
