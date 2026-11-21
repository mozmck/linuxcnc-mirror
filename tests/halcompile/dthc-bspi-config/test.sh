#!/bin/sh
# Simulates ccc_dthc_bspi against a fake DTHC6 to check that THC settings
# edited on the screen (Preset Volts) reach the board, and that a stale
# configuration readback never sends an older value back.
set -e
HERE=$(pwd)
T=$(mktemp -d)
trap 'rm -rf "$T"' EXIT
cp "$HERE/../../../src/hal/drivers/ccc_dthc_bspi.comp" "$HERE/dthc_bspi_config_sim.c" "$T/"
cd "$T"
halcompile --preprocess ccc_dthc_bspi.comp >/dev/null
cat > sim.mk <<MK
include $(halcompile --print-modinc)
sim: dthc_bspi_config_sim.c ccc_dthc_bspi.c
	\$(CC) \$(EXTRA_CFLAGS) -w -DCOMP_C='"ccc_dthc_bspi.c"' -o \$@ dthc_bspi_config_sim.c -lm
MK
make -s -f sim.mk sim
./sim
./sim motor
