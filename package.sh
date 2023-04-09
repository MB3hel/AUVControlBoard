#!/usr/bin/env bash

# Exit if any command errors
set -e
trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "\"${last_command}\" command filed with exit code $?."' EXIT

read -p "Version: " VERSION

# Work in same directory as this script
DIR=$(realpath $(dirname "$0"))
cd "$DIR"

# Base directory structure
rm -rf package/
mkdir package/
mkdir package/build/
mkdir package/build/v1/
mkdir package/build/v2/
mkdir package/python/

# Build firmware
cd firmware
rm -rf build/
cmake --preset v1
cmake --build --preset v1-release
cmake --preset v2
cmake --build --preset v2-release
cd ..

# Copy firmware files
cp -r firmware/build/v1/Release/ package/build/v1/
cp -r firmware/build/v2/Release/ package/build/v2/
cp firmware/flash.py package/
cp firmware/reboot_bootloader.py package/

# Copy Python interface code
cp iface/control_board.py package/python/

# Create zip packge
cd package
rm -f ./AUVControlBoard_${VERSION}.zip
zip -r ../AUVControlBoard_${VERSION}.zip .
mv ../AUVControlBoard_${VERSION}.zip .
