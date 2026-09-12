#!/usr/bin/env bash
set -euo pipefail
cmake -S . -B build -DCMAKE_CXX_COMPILER="${CXX:-g++}"
cmake --build build
ctest --test-dir build --output-on-failure
