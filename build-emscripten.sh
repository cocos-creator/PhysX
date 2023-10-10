#!/bin/bash
# emscripten 3.1.41

copy_dest=../../cocos-engine/native/external/emscripten/physx

set -e

echo -e "\033[01;32m |||  START  ||| \033[0m"

get_current_time_in_seconds() {
    local now=$(date +'%Y-%m-%d %H:%M:%S')
    local total_seconds
    if [[ "$OSTYPE" == "darwin"* ]]; then
        total_seconds=$(date -j -f "%Y-%m-%d %H:%M:%S" "$now" "+%s")
    else
        total_seconds=$(date --date="$now" +%s)
    fi
    echo "$total_seconds"
}

start_time=$(get_current_time_in_seconds)

base_dir=$(cd "$(dirname "$0")";pwd)
mode="release"
if [ $1 ]; then mode=$1; fi

if [[ "$OSTYPE" == "msys"* || "$OSTYPE" == "cygwin" ]]; then
    script_suffix='.bat'
else
    script_suffix='.sh'
fi

cd $base_dir
mkdir -p $base_dir/builds

# print mode
echo -e "\033[01;32m |||  MODE: $mode  ||| \033[0m"

#-------------------------asm.js-----------------------------------
# if mode is release, build asm.js
if [ $mode == "release" ]; then
echo -e "\033[01;32m |||  BUILD ASM.JS START  ||| \033[0m"
echo -e "\033[01;32m |||  GENERATE ||| \033[0m"
cd $base_dir
cd physx/
./generate_projects${script_suffix} emscripten-js

if [ $? -eq 0 ]; then
    echo -e "\033[01;32m Generated Project successfully \033[0m"
else
    echo -e "\033[01;32m Failed to generate Project \033[0m"
    exit 1
fi

echo -e "\033[01;32m |||  COMPILE ||| \033[0m"
cd compiler/emscripten-js-$mode
# ninja

echo -e "\033[01;32m |||  BUILD ASM.JS END  ||| \033[0m"
cd $base_dir
cp $base_dir/physx/bin/emscripten/$mode/physx.$mode.asm.js $base_dir/builds
if [ -d "$copy_dest" ]; then
    cp -r $base_dir/builds/physx.$mode.asm.js $copy_dest
fi
fi

#--------------------------wasm----------------------------------
echo -e "\033[01;32m |||  BUILD WASM START  ||| \033[0m"
echo -e "\033[01;32m |||  GENERATE ||| \033[0m"
cd $base_dir
cd physx/
./generate_projects${script_suffix} emscripten-wasm

if [ $? -eq 0 ]; then
    echo -e "\033[01;32m Generated Project successfully \033[0m"
else
    echo -e "\033[01;32m Failed to generate Project \033[0m"
    exit 1
fi

echo -e "\033[01;32m |||  COMPILE ||| \033[0m"
cd compiler/emscripten-wasm-$mode
# ninja

echo -e "\033[01;32m |||  BUILD WASM END  ||| \033[0m"
cd $base_dir
cp $base_dir/physx/bin/emscripten/$mode/physx.$mode.wasm.js $base_dir/builds
cp $base_dir/physx/bin/emscripten/$mode/physx.$mode.wasm.wasm $base_dir/builds
if [ -d "$copy_dest" ]; then
    cp -r $base_dir/builds/physx.$mode.wasm.js $copy_dest
    cp -r $base_dir/builds/physx.$mode.wasm.wasm $copy_dest
fi

#------------------------------------------------------------
echo -e "\033[01;32m |||  END  |||  \033[0m"
end_time=$(get_current_time_in_seconds)
echo -e "\033[01;32m Total Time Used: "$((end_time-start_time))"s  \033[1m"
