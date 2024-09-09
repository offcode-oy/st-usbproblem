#!/bin/bash

find . -type f -name "*.c" -exec dos2unix {} \;
find . -type f -name "*.h" -exec dos2unix {} \;
find . -type f -name "*.txt" -exec dos2unix {} \;
find . -type f -name "LICENSE" -exec dos2unix {} \;
find . -type f -name ".mxproject" -exec dos2unix {} \;
find . -type f -name "postbuild.sh" -exec dos2unix {} \;
