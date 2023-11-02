# Exit if any command errors
set -e
trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "\"${last_command}\" command filed with exit code $?."' ERR

# Work in same directory as this script
DIR=$(realpath $(dirname "$0"))
cd "$DIR"

# Base directory structure
rm -rf pack/firmware
mkdir pack/firmware/
mkdir pack/firmware/tools/
mkdir pack/firmware/build/
mkdir pack/firmware/build/v1/
mkdir pack/firmware/build/v2/
mkdir pack/firmware/source/

# Build firmware
pushd ../firmware > /dev/null
rm -rf build/
cmake --preset v1
cmake --build --preset v1-release
cmake --preset v2
cmake --build --preset v2-release
popd

# Copy firmware files
cp -r ../firmware/build/v1/Release/ pack/firmware/build/v1/
cp -r ../firmware/build/v2/Release/ pack/firmware/build/v2/
cp ../firmware/flash.py pack/firmware/
cp ../firmware/reboot_bootloader.py pack/firmware/
cp ../firmware/COPYING pack/firmware/
cp -r ../firmware/tools/uf2conv/ pack/firmware/tools/

cp -r ../firmware/src/ pack/firmware/source/
cp -r ../firmware/include/ pack/firmware/source/
cp -r ../firmware/thirdparty/ pack/firmware/source/
cp -r ../firmware/toolchains/ pack/firmware/source/
cp ../firmware/CMakeLists.txt pack/firmware/source/
cp ../firmware/CMakePresets.json pack/firmware/source/
