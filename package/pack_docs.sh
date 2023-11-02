#!/usr/bin/env bash

# Exit if any command errors
set -e
trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "\"${last_command}\" command filed with exit code $?."' ERR

# Work in same directory as this script
DIR=$(realpath $(dirname "$0"))
cd "$DIR"

# Remove old content if any
rm -rf pack/docs

# Create docs site
pushd ../docs > /dev/null
rm -rf site
SITE_URL="" DIR_URLS=false mkdocs build
popd

# Copy docs site
cp -r ../docs/site pack/docs/
