#!/bin/sh
find . -type f -name '*.h' -o -iname '*.c' -o -iname '*.cpp' -o -iname '*.hpp' -o -iname '*.ino' \
    | grep -v -e mls_index.c -e build/ -e esp32-camera/ -e esp-dsp/ \
    | xargs clang-format --verbose -style=file -i -fallback-style=none

yapf --verbose -ri --exclude 'submodules/**/*.py' .
