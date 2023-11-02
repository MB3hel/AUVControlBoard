#!/usr/bin/env sh
set -e
DIR=`dirname "$0"`
has_port=0
while [ $has_port -eq 0 ]; do
read -p "TCP Port for SimCB: " port
    case $port in
        ''|*[!0-9]*) echo "Invalid port. Must be a (positive) number!"; continue ;;
    esac
    if [ $port -gt 65535 ]; then
        echo "Invalid port. Must be less than or equal to 65535!"
    elif [ $port -lt 0 ]; then
        echo "Invalid port. Must be greater than or equal to 0!;"
    else
        has_port=1
    fi
done
echo "Running SimCB on port $port"
"$DIR"/SimCB $port