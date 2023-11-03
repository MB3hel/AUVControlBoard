#!/usr/bin/env bash

# Work in same directory as this script
DIR=$(realpath $(dirname "$0"))
cd "$DIR"

# Remove pack directory
rm -rf pack/

# Remove tmp direcotry
rm -rf tmp/