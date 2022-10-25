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

if [[ "$0" == *build-lfc ]]; then
  echo -e "\033[33mWarning; buid-lfc is deprecated! Please use build-lf-cli instead.\033[0m"
fi

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
    source "${base}/${rel_path}/include.sh"
else
    fatal_error "Unable to determine absolute path to $0."
fi
#============================================================================

# Check whether sources are present and exit if they are not.
if [ ! "$(get_src_dir)" ]; then
    fatal_error "Cannot find the Lingua Franca sources."
fi

# Print message explaining the CLI args.
function usage() {
    echo "Usage: build-lf-cli [options] [lfc-args]]"
    echo "Options:"
    echo "  -c | --clean          Build entirely from scratch."
    echo "  -h | --help           Display this information."
    echo "  -o | --offline        Use cached libraries."
    echo "  -s | --stacktrace     Provide stacktrace of build errors."
}

flags=" ";
clean=false
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
            clean=true
        ;;
        -h | --help )
            usage
            exit 0
        ;;
        *) 
            usage
            exit 1
        ;;
    esac
    shift
done

# Perform cleanup up if requested.
if [[ "${clean}" == "true" ]]; then
    echo "Performing cleanup..."
    "${base}/gradlew" -p "${base}" clean;
fi

# Check if jar is missing or out-of-date compared to the sources.
find_cmd=find
# Do not use the built-in Windows 'find' command, which is different.
if [ "${OSTYPE}" = "msys" ]; then
	find_cmd="/usr/bin/${FIND}"
fi

jar_path="$(get_jar_path)"

if [ ! -f "${jar_path}" ] || ! "${find_cmd}" "${base}" \
        -path "${src_pkg_path}" \
        -prune -o \
        -type f \
        -newer "${jar_path}" \
        -exec false {} +; then
	# Rebuild.
    1>&2 echo "Jar file is missing or out-of-date; starting rebuild..."
	"${base}/gradlew" ${flags} -p "${base}" buildAll
	# Update the timestamp in case the jar was not touched by Gradle.
    touch -c -- "${jar_path}"
else
    echo "Already up-to-date."
fi
