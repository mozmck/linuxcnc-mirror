#!/bin/sh
# Simulates ccc_dthc_bspi on an EBOB BSPI with hm2_eth timing and a fake
# DTHC6, to check that the replies realign after Ethernet packet errors and
# servo-thread latency instead of failing SOF for good.
set -e
HERE=$(pwd)
T=$(mktemp -d)
trap 'rm -rf "$T"' EXIT
cp "$HERE/../../../src/hal/drivers/ccc_dthc_bspi.comp" "$HERE/dthc_bspi_resync_sim.c" "$T/"
cd "$T"
halcompile --preprocess ccc_dthc_bspi.comp >/dev/null
cat > sim.mk <<MK
include $(halcompile --print-modinc)
sim: dthc_bspi_resync_sim.c ccc_dthc_bspi.c
	\$(CC) \$(EXTRA_CFLAGS) -w -DCOMP_C='"ccc_dthc_bspi.c"' -o \$@ dthc_bspi_resync_sim.c -lm
MK
make -s -f sim.mk sim
./sim
