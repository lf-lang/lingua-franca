#!/bin/bash

#============================================================================
# Description:      Visualize federated trace data for RTI-federate interactions.
# Authors:          Chadlia Jerad
#                   Edward A. Lee
# Usage:            Usage: fedsd -r [rti.csv] -f [fed.csv ...]
#============================================================================

#============================================================================
# Preamble
#============================================================================

# Copied from build.sh    FIXME: How to avoid copying

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
    1>&2 echo -e "\e[1mfedsd: \e[31mfatal error: \e[0m$1"
    exit 1
}

abs_path="$(find_dir "$0")"

if [[ "${abs_path}" ]]; then
    base=`dirname $(dirname ${abs_path})`
else
    fatal_error "Unable to determine absolute path to $0."
fi

# Get the lft files
lft_files_list=$@

if [ -z "$lft_files_list" ]
then
    echo "Usage: fedsd [lft files]"
    exit 1
fi

# Initialize variables
csv_files_list=''
extension='.csv'
rti_csv_file=''

# Iterate over the lft file list to:
#  - First, transform into csv
#  - Second, construct the csv fiel name
#  - Then construct the csv file list
# The csv file list does include the rti, it is put in a separate variable
for each_lft_file in $lft_files_list
    do
        # Tranform to csv
        ${base}/bin/trace_to_csv $each_lft_file
        # Get the file name
        csv=${each_lft_file%.*}
        if [ $csv == 'rti' ]
        then
            # Set the rti csv file
            rti_csv_file='rti.csv'
        else
            # Construct the csv file name and add it to the list
            csv_files_list="$csv$extension $csv_files_list"
        fi
    done

# echo $lft_files_list
# echo $rti_csv_file
# echo $csv_files_list

# FIXME: Check that python3 is in the path.
if [ ! -z $rti_csv_file ]
then
    python3 "${base}/util/tracing/visualization/fedsd.py" "-f" $csv_files_list
else
    echo Building the communication diagram for the following trace files: $lft_files_list in trace_svg.html
    python3 "${base}/util/tracing/visualization/fedsd.py" "-r" "$rti_csv_file" "-f" $csv_files_list
fi
