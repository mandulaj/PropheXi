#!/bin/bash

# exit when any command fails
set -e -o pipefail

mkdir -p build
pushd build
cmake .. -G "Unix Makefiles"
make
popd
