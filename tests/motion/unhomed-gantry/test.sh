#!/bin/bash
set -eu
ulimit -c 0
root=$(cd "$(dirname "$0")/../../.." && pwd)
build=$(mktemp -d)
trap 'rm -rf "$build"' EXIT
cd "$root"
includes=(-Isrc -Isrc/emc/motion -Isrc/emc/kinematics -Isrc/emc/tp
          -Isrc/emc/nml_intf -Isrc/emc/ini -Isrc/libnml/posemath
          -Isrc/hal -Isrc/rtapi -Iinclude -I/usr/include/linuxcnc)
flags=(-O1 -g -ffunction-sections -fdata-sections -fsanitize=undefined,bounds
       -fno-sanitize-recover=all)
"${CC:-gcc}" -std=gnu99 -DRTAPI -DUSPACE "${flags[@]}" "${includes[@]}" \
    tests/motion/unhomed-gantry/test-control.c src/emc/kinematics/kins_util.c \
    src/emc/kinematics/cubic.c -Wl,--gc-sections -lm -o "$build/control"
"$build/control"
"${CXX:-g++}" -std=gnu++11 -DULAPI "${flags[@]}" "${includes[@]}" \
    tests/motion/unhomed-gantry/test-position.cc -Llib \
    -Wl,--gc-sections -llinuxcncini -o "$build/position"
"$build/position" "$build"
