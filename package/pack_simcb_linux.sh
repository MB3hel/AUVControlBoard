#!/usr/bin/env bash

# Exit if any command errors
set -e
trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "\"${last_command}\" command filed with exit code $?."' ERR

# Work in same directory as this script
DIR=$(realpath $(dirname "$0"))
cd "$DIR"

# Base directory structure
rm -rf pack/simcb/linux-amd64/
if [ ! -d pack/simcb ]; then
    mkdir pack/simcb/
fi
mkdir pack/simcb/linux-amd64/

# Build firmware
pushd ../firmware > /dev/null
rm -rf build/
cmake --preset simcb-linux
cmake --build --preset simcb-linux-release
popd

# Copy firmware files
cp -r ../firmware/build/simcb-linux/Release/SimCB pack/simcb/linux-amd64/
cp ./simcb-linux-launch.sh pack/simcb/linux-amd64/SimCB.sh
mv pack/simcb/linux-amd64/SimCB pack/simcb/linux-amd64/_SimCB
chmod +x pack/simcb/linux-amd64/SimCB.sh
