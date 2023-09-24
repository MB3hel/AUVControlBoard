#!/usr/bin/env bash

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
echo "    space = full release"
echo "  If type is specified (not space), also provide a build number."
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
" ") ;;
*) echo "Invalid type!"; exit 1;;
esac

if [ "$VER_TYPE" != " " ]; then
    read -p "Build: " VER_BUILD
    VERSION="$VER_MAJOR.$VER_MINOR.$VER_REV-$VER_TYPE_FULL$VER_BUILD"
else
    VER_BUILD="0"
    VERSION="$VER_MAJOR.$VER_MINOR.$VER_REV"
fi
echo ""


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

# Update version in firmware code
sed -i "s/#define FW_VER_MAJOR.*/#define FW_VER_MAJOR $VER_MAJOR/g" firmware/include/metadata.h
sed -i "s/#define FW_VER_MINOR.*/#define FW_VER_MINOR $VER_MINOR/g" firmware/include/metadata.h
sed -i "s/#define FW_VER_REVISION.*/#define FW_VER_REVISION $VER_REV/g" firmware/include/metadata.h
sed -i "s/#define FW_VER_TYPE.*/#define FW_VER_TYPE '$VER_TYPE'/g" firmware/include/metadata.h
sed -i "s/#define FW_VER_BUILD.*/#define FW_VER_BUILD $VER_BUILD/g" firmware/include/metadata.h

# Write version file
echo "$VERSION" > package/version.txt

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
