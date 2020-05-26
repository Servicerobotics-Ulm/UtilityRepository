#!/bin/bash
echo $PWD
git clone https://chromium.googlesource.com/chromium/tools/depot_tools.git ../depot_tools
export PATH=$PATH:../depot_tools
cd ../
rm -rf breakpad
mkdir breakpad
cd breakpad
fetch breakpad
#./configure && make
#make install
