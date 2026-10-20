#!/bin/sh
# Builds and runs the FOC simulation on the PC (needs gcc only).
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
CORE="$HERE/../Core"
OUT=${TMPDIR:-/tmp}/foc_sim
gcc -std=gnu11 -O2 -Wall -I"$CORE/Inc" "$HERE/foc_sim.c" "$CORE/Src/esc_foc.c" "$CORE/Src/esc_params.c" -o "$OUT" -lm
"$OUT" "$@"
