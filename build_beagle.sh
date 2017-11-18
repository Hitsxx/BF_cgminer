#!/bin/sh

# build for ARM

./configure --enable-bitfury16 --enable-cpumining --host=arm-linux-gnueabihf --disable-libcurl
make
arm-linux-gnueabi-strip cgminer
