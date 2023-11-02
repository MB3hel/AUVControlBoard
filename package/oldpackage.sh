#!/usr/bin/env bash

# Exit if any command errors
set -e
trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "\"${last_command}\" command filed with exit code $?."' ERR

# Work in same directory as this script
DIR=$(realpath $(dirname "$0"))
cd "$DIR"

# Base directory structure
rm -rf package/
mkdir package/
mkdir package/firmware/
mkdir package/firmware/tools/
mkdir package/firmware/build/
mkdir package/firmware/build/v1/
mkdir package/firmware/build/v2/
mkdir package/firmware/source/
mkdir package/iface/
mkdir package/iface/scripts/
mkdir package/iface/example/

# Build firmware
cd firmware
rm -rf build/
cmake --preset v1
cmake --build --preset v1-release
cmake --preset v2
cmake --build --preset v2-release
cd ..

# Copy firmware files
cp -r firmware/build/v1/Release/ package/firmware/build/v1/
cp -r firmware/build/v2/Release/ package/firmware/build/v2/
cp firmware/flash.py package/firmware/
cp firmware/reboot_bootloader.py package/firmware/
cp firmware/COPYING package/firmware/
cp -r firmware/tools/uf2conv/ package/firmware/tools/

cp -r firmware/src/ package/firmware/source/
cp -r firmware/include/ package/firmware/source/
cp -r firmware/thirdparty/ package/firmware/source/
cp -r firmware/toolchains/ package/firmware/source/
cp firmware/CMakeLists.txt package/firmware/source/
cp firmware/CMakePresets.json package/firmware/source/



# Copy Python interface code
cp iface/*.py package/iface/
cp iface/scripts/*.py package/iface/scripts/
cp iface/example/*.py package/iface/example/
cp iface/COPYING package/iface/COPYING

# Create docs site
cd docs
rm -rf site
SITE_URL="" DIR_URLS=false mkdocs build
cd ..

# Copy docs site
cp -r docs/site package/docs/

# Create zip packge
cd package
rm -f ./AUVControlBoard_${VERSION}.zip
zip -r ../AUVControlBoard_${VERSION}.zip .
mv ../AUVControlBoard_${VERSION}.zip .
