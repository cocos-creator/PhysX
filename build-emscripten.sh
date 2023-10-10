#!/bin/bash
# emscripten 3.1.41

set -e

echo -e "\033[01;32m --------------- START -------------------- \033[0m"

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

echo "--------|||  BUILD JS  |||--------"
echo "|||  GENERATE |||"
cd physx/
./generate_projects${script_suffix} emscripten-js

if [ $? -eq 0 ]; then
    echo "Generated Project successfully"
else
    echo "Failed to generate Project"
    exit 1
fi

echo "|||  COMPILE |||"
cd compiler/emscripten-js-$mode
ninja

echo "--------|||  BUILD WASM  |||--------"
cd $base_dir
echo "|||  GENERATE |||"
cd physx/
./generate_projects${script_suffix} emscripten-wasm
echo "|||  COMPILE |||"
cd compiler/emscripten-wasm-$mode
ninja

echo "|||  COPY  |||"
cd $base_dir
mkdir -p $base_dir/builds
cp $base_dir/physx/bin/emscripten/$mode/physx.$mode.asm.js $base_dir/builds/physx.$mode.asm.js
cp $base_dir/physx/bin/emscripten/$mode/physx.$mode.wasm.js $base_dir/builds/physx.$mode.wasm.js
cp $base_dir/physx/bin/emscripten/$mode/physx.$mode.wasm.wasm $base_dir/builds/physx.$mode.wasm.wasm

engineDir="../../cocos-engine/native/external/emscripten/physx"
if [ -d "$engineDir" ]; then
    cp -r $base_dir/builds/physx.$mode.asm.js $engineDir/physx.$mode.asm.js
    cp -r $base_dir/builds/physx.$mode.wasm.js $engineDir/physx.$mode.wasm.js
    cp -r $base_dir/builds/physx.$mode.wasm.wasm $engineDir/physx.$mode.wasm.wasm
fi

echo "|||  FINISH  |||"

end_time=$(get_current_time_in_seconds)
echo -e "\033[01;32m Time Used: "$((end_time-start_time))"s  \033[1m"
echo -e "\033[01;32m ------------- END -----------------  \033[0m"

