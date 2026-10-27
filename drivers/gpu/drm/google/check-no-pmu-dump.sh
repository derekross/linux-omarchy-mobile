#!/bin/bash
# SPDX-License-Identifier: GPL-2.0-only

# Do not enable set -e. grep returns 1 when it finds nothing, and that
# result is a pass. The address is split so this file does not contain it.

dir=$(CDPATH='' cd -- "$(dirname -- "$0")" && pwd) || exit 2
needle=$(printf '0x%s%s' '1546' '1a80')

grep -R -n -I -F -- "$needle" "$dir"
status=$?
if [[ $status -eq 0 ]]; then
	exit 1
fi
if [[ $status -eq 1 ]]; then
	exit 0
fi
exit "$status"
