#!/bin/bash

#============================================================================
# Description: 	Build the Lingua Franca compiler.
# Authors:		Marten Lohstroh, Mehrdad Niknami, Christian Menard
# Usage:		build-lfc [options] [[-r | --run] [lfc-args]]
#============================================================================

#============================================================================
# Preamble
#============================================================================
# Find the directory in which this script resides in a way that is compatible
# with MacOS, which has a `readlink` implementation that does not support the
# necessary `-f` flag to canonicalize by following every symlink in every 
# component of the given name recursively.
# This solution, adapted from an example written by Geoff Nixon, ia POSIX-
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

rel_path="/lib/scripts"
abs_path="$(find_dir "$0")"

if [[ "${abs_path}" ]]; then
    base=`dirname $(dirname ${abs_path})`
    # Initialize Lingua Franca shell environment.
    source "${base}/${rel_path}/init.sh"
else
    fatal_error "Unable to determine absolute path to $0."
fi
#============================================================================

# 1. Find the jar and check whether sources are present or not.
find_jar_path

if ! src_exists; then
    fatal_error "Cannot find sources."
fi

# 2. Print message explaining the CLI args.
function usage() {
    echo "Usage: build-lfc [options] [[-r | --run] [lfc-args]]"
    echo "Options:"
    echo "  -c | --clean          Build entirely from scratch."
    echo "  -h | --help           Display this information."
    echo "  -o | --offline        Use cached libraries."
    echo "  -s | --stacktrace     Provide stacktrace of build errors."
}

flags=" ";
clean=0;
run=false
args=()
while [[ "$#" -gt 0 ]]; do 
    case $1 in
        -o | --offline ) 
            flags=$flags"--offline "
        ;;
        -s | --stacktrace ) 
            flags=$flags"--stacktrace "
        ;;
        -c | --clean )
            clean=1
        ;;
        -r | --run ) 
            run=true
            shift
            while [[ "$#" -gt 0 ]]; do
                args+=("$1")
                shift
            done
            break
        ;;
        *) 
            usage
            exit 1
        ;;
    esac
    shift
done

# 3. Clean up if requested.
if [ $clean -eq 1 ]; then
    echo "Performing cleanup..."
    "${base}/gradlew" -p "${base}" clean;
fi

# 4. Check if jar is missing or out-of-date compared to the sources
if [ ! -f "${lfc_jar_build_path}" ] || ! "${FIND}" "${lfbase}/src" -path "${lfbase}/test" -prune -o -type f -newer "${lfc_jar_build_path}" -exec false {} +; then
	1>&2 echo "Jar file is missing or out-of-date; running Gradle..."
	"${base}/gradlew" ${flags} -p "${base}" buildLfc
	touch -c -- "${jar_path}"  # Ensure the file timestamp is up-to-date even if the file didn't need to be updated
else
    echo "Already up-to-date."
fi

# 5. Run lfc with the provided arguments.
if [[ "${run}" == "true" ]]; then
    check_jre_version
    echo "Running lfc..."
    run_jar_with_args "${args[@]}"
fi
