#!/bin/bash
# Host tests of the controller code (HAL stand-ins in Tests/stubs/, see below).
#
# Compiles and runs every Controller/Tests/*_test.c with the host C compiler (CC, default cc).
# A test lists the firmware sources it needs (paths relative to Controller/) in one line
#   // test-sources: Core/Src/a.c Core/Src/b.c
# Tests/stubs/ is on the include path: host stand-ins for HAL headers (stm32f4xx_hal.h with the GPIO
# declarations of control_logic.c).
# Generated files are written to a temporary build folder that is on the include path:
#   control_logic_mapping.inc  the phase-to-torque mapping block of Core/Src/control_logic.c
#                              (from "float gaitCycle = *gaitCycleNorm;" to the end of the
#                              segment loop), so the test runs the firmware's code as it is.
# Exit status: 0 if every test compiled and passed.

CC=${CC:-cc}
HERE=$(cd "$(dirname "$0")" && pwd)
CTRL=$(dirname "$HERE")
BUILD=$(mktemp -d "${TMPDIR:-/tmp}/controller_tests.XXXXXX") || exit 1
trap 'rm -rf "$BUILD"' EXIT
CFLAGS=(-std=gnu11 -O2 -Wall -Wextra -ffp-contract=off -I"$CTRL/Core/Inc" -I"$HERE/stubs" -I"$BUILD")

awk '
	index($0, "float gaitCycle = *gaitCycleNorm;") { on = 1 }
	on {
		print
		if ($0 ~ /for \(/) inloop = 1
		if (inloop) {
			depth += gsub(/\{/, "{") - gsub(/\}/, "}")
			if (depth > 0) opened = 1
			if (opened && depth == 0) exit
		}
	}' "$CTRL/Core/Src/control_logic.c" > "$BUILD/control_logic_mapping.inc"
if ! grep -q 'evaluate_hermite' "$BUILD/control_logic_mapping.inc"; then
	echo "run_tests: mapping block not found in Core/Src/control_logic.c" >&2
	exit 1
fi

PASS=0; FAIL=0
for t in "$HERE"/*_test.c; do
	[ -e "$t" ] || continue
	name=$(basename "$t" .c)
	srcs=()
	for s in $(sed -n 's#^// test-sources:##p' "$t"); do srcs+=("$CTRL/$s"); done
	echo "== $name"
	if "$CC" "${CFLAGS[@]}" -o "$BUILD/$name" "$t" ${srcs[@]+"${srcs[@]}"} -lm && "$BUILD/$name"; then
		PASS=$((PASS + 1))
	else
		echo "== $name: FAILED"
		FAIL=$((FAIL + 1))
	fi
done
echo "== tests passed: $PASS, failed: $FAIL"
[ $FAIL -eq 0 ] && [ $PASS -gt 0 ]
