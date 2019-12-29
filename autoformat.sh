#!/bin/sh
find . \
-type d \( -path ./esp32_cam_app/components -o -path ./esp32_cam_app/build -o -path ./esp32_cam_app/main/build \) -prune -o \
-name *.h -o -iname *.c -o -iname *.cpp -o -iname *.hpp \
    | xargs clang-format --verbose -style=file -i -fallback-style=none

yapf --verbose -ri .
