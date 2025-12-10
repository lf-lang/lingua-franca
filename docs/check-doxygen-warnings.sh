#!/bin/bash
# Script to check Doxygen warnings and filter out known false positives
# Exit code: 0 if no real warnings, 1 if there are real warnings

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Run Doxygen and capture warnings
DOXYGEN_OUTPUT=$(doxygen Doxyfile.in 2>&1 || true)

# Filter out known false positives:
# 1. Generic type parameter warnings - these are false positives due to Doxygen's
#    limitation in parsing Java generics, even with INPUT_FILTER preprocessing
# 2. Anonymous inner class @param warnings - Doxygen incorrectly associates @param tags
#    from inner class methods with the outer method

FILTERED_WARNINGS=$(echo "$DOXYGEN_OUTPUT" | grep "warning:" | \
  grep -v "argument '[A-Z]' of command @param is not found" | \
  grep -v "PythonValidator.getPossibleStrategies has @param documentation sections but no arguments" || true)

# Count remaining warnings (handle empty string case)
if [ -z "$FILTERED_WARNINGS" ]; then
  WARNING_COUNT=0
else
  WARNING_COUNT=$(echo "$FILTERED_WARNINGS" | grep -c "warning:" || echo "0")
fi

if [ "$WARNING_COUNT" -gt 0 ]; then
  echo "ERROR: Doxygen found $WARNING_COUNT real warning(s) (false positives filtered out):"
  echo ""
  echo "$FILTERED_WARNINGS"
  echo ""
  echo "Please fix these warnings before merging."
  exit 1
else
  echo "OK: No Doxygen warnings found (false positives filtered out)."
  exit 0
fi
