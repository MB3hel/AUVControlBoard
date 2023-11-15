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
mkdir -p pack/simcb/linux-amd64/

# Build in docker container
# Ensures broad glibc compatability as building on an older system container
rm -rf tmp/out/
mkdir -p tmp/
cp -r ../firmware/ tmp/     # Can only copy from child directories of dockerfile, so can't COPY ../firmware
docker build --output tmp/out/ -f pack_simcb_linux_dockerfile . 

# Copy out result
cp -r tmp/out/src/firmware/build/simcb-linux/Release/SimCB ./pack/simcb/linux-amd64/
cp ./simcb-linux-launch.sh pack/simcb/linux-amd64/SimCB.run
chmod +x pack/simcb/linux-amd64/SimCB.run
rm -rf tmp/out/
