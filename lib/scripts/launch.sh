#!/bin/bash

#============================================================================
# Description:      Run the Lingua Franca compiler.
# Authors:          Marten Lohstroh
#                   Christian Menard
# Usage:            Usage: lfc [options] files...
#============================================================================

#============================================================================
# Preamble
#============================================================================

# Find the directory in which this script resides in a way that is compatible
# with MacOS, which has a `readlink` implementation that does not support the
# necessary `-f` flag to canonicalize by following every symlink in every
# component of the given name recursively.
# This solution, adapted from an example written by Geoff Nixon, is POSIX-
# compliant and robust to symbolic links. If a chain of more than 1000 links
# is encountered, we return.
find_dir() (
  start_dir=$PWD
  cd "$(dirname "$1")"
  link=$(readlink "$(basename "$1")")
  count=0
  while [ "${link}" ]; do
    if [[ "${count}" -lt 1000 ]]; then
        cd "$(dirname "${link}")"
        link=$(readlink "$(basename "$1")")
        ((count++))
    else
        return
    fi
  done
  real_path="$PWD/$(basename "$1")"
  cd "${start_dir}"
  echo `dirname "${real_path}"`
)

# Report fatal error and exit.
function fatal_error() {
    1>&2 echo -e "\e[1mlfc: \e[31mfatal error: \e[0m$1"
    exit 1
}

rel_path="lib/scripts"
abs_path="$(find_dir "$0")"

if [[ "${abs_path}" ]]; then
    base=`dirname $(dirname ${abs_path})`
    source "${base}/${rel_path}/include.sh"
else
    fatal_error "Unable to determine absolute path to $0."
fi
#============================================================================


if [[ "$0" == *lfc ]]; then
  main_class="org.lflang.cli.Lfc"
elif [[ "$0" == *lff ]]; then
  main_class="org.lflang.cli.Lff"
else
  known_commands="[lfc, lff]"
  echo \
  "ERROR: $0 is not a known lf command! Known commands are ${known_commands}.
       In case you use a symbolic or hard link to one of the Lingua Franca
       command line tools, make sure that the link's name ends with one of
       ${known_commands}."
  exit 2
fi

# Launch the compiler.
run_cli_tool_with_args "$@"
