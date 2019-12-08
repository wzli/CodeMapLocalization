#!/bin/sh
find . \
-type d \( -path ./esp32_cam_app/components -o -path ./esp32_cam_app/build \) -prune -o \
-name *.h -o -iname *.c -o -iname *.cpp -o -iname *.hpp \
    | xargs clang-format -style=file -i -fallback-style=none

yapf -ri .
