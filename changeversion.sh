#!/usr/bin/env bash

# Work in same directory as this script
DIR=$(realpath $(dirname "$0"))
cd "$DIR"

# Exit if any command errors
set -e
trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "\"${last_command}\" command filed with exit code $?."' ERR

# Get version info from user
echo "Enter version info"
echo "  [MAJOR].[MINOR].[REVISION]-[TYPE][BUILD]"
echo "  For type, use the following: "
echo "    a = alpha"
echo "    b = beta"
echo "    c = release candidate (rc)"
echo "    - = full release"
echo "  If type is specified (not hyphen), also provide a build number."
echo "  Nubmers (major, minor, revision, build) must be 0-255 (inclusive)"
echo ""
read -p "Major: " VER_MAJOR
read -p "Minor: " VER_MINOR
read -p "Revision: " VER_REV
read -p "Type: " VER_TYPE

case "$VER_TYPE" in
"a") VER_TYPE_FULL="alpha";;
"b") VER_TYPE_FULL="beta";;
"c") VER_TYPE_FULL="rc";;
"-") VER_TYPE=" "; VER_TYPE_FULL="";;
*) echo "Invalid type!"; exit 1;;
esac

if [ "$VER_TYPE" != " " ]; then
    read -p "Build: " VER_BUILD
    VERSION="$VER_MAJOR.$VER_MINOR.$VER_REV-$VER_TYPE_FULL$VER_BUILD"
    VERSION_SHORT="$VER_MAJOR.$VER_MINOR.$VER_REV-$VER_TYPE$VER_BUILD"
else
    VER_BUILD=0
    VERSION="$VER_MAJOR.$VER_MINOR.$VER_REV"
    VERSION_SHORT="$VER_MAJOR.$VER_MINOR.$VER_REV"
fi
echo ""

# Update version in firmware code
sed -i "s/#define FW_VER_MAJOR.*/#define FW_VER_MAJOR $VER_MAJOR/g" firmware/include/metadata.h
sed -i "s/#define FW_VER_MINOR.*/#define FW_VER_MINOR $VER_MINOR/g" firmware/include/metadata.h
sed -i "s/#define FW_VER_REVISION.*/#define FW_VER_REVISION $VER_REV/g" firmware/include/metadata.h
sed -i "s/#define FW_VER_TYPE.*/#define FW_VER_TYPE '$VER_TYPE'/g" firmware/include/metadata.h
sed -i "s/#define FW_VER_BUILD.*/#define FW_VER_BUILD $VER_BUILD/g" firmware/include/metadata.h

# Update version in interface script
sed -i "s/VER_STR = .*/VER_STR = \"$VERSION_SHORT\"/g" iface/control_board.py

# Write version file
echo "$VERSION" > package/version.txt