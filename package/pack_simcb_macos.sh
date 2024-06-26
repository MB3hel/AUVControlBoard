#!/usr/bin/env bash

# Exit if any command errors
set -e
trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "\"${last_command}\" command filed with exit code $?."' ERR

# Work in same directory as this script
DIR=$(dirname "$0")
cd "$DIR"

# Base directory structure
rm -rf pack/simcb/macos-intel/
mkdir -p pack/simcb/macos-intel/

# Build firmware
pushd ../firmware > /dev/null
rm -rf build/
cmake --preset simcb-macos
cmake --build --preset simcb-macos-release
popd

# Copy firmware files
cp -r ../firmware/build/simcb-macos/Release/SimCB pack/simcb/macos-intel/
