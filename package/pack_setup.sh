#!/usr/bin/env bash

# Work in same directory as this script
DIR=$(realpath $(dirname "$0"))
cd "$DIR"

# Remove pack directory
mkdir pack/
cp version.txt pack/