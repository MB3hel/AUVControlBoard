#!/usr/bin/env bash

# Work in same directory as this script
DIR=$(realpath $(dirname "$0"))
cd "$DIR"

# Setup directory structure
rm -rf pack/iface/
mkdir pack/iface/
mkdir pack/iface/scripts/
mkdir pack/iface/example/

# Copy Python interface code
cp ../iface/*.py pack/iface/
cp ../iface/scripts/*.py pack/iface/scripts/
cp ../iface/example/*.py pack/iface/example/
cp ../iface/COPYING pack/iface/COPYING
