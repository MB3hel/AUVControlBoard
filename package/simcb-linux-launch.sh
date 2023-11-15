#!/usr/bin/env sh
set -e
DIR=`dirname "$0"`
chmod +x "$DIR"/SimCB
"$DIR"/SimCB "$@"