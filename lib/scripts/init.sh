#!/bin/bash

#============================================================================
# Description:  Initialize an environment for Lingua Franca helper scripts.
# Authors:		Marten Lohstroh, Mehrdad Niknami, Christian Menard
# Usage:		Source this file at the beginning of other scripts to access
#               the functions and variables defined in this script.
#============================================================================

# Set up the environment.

# -e: Immediately exit if any command has a non-zero exit status.
# -u: Error on referencing unset variables. 
# -o pipefail: Prevents errors in a pipeline from being masked.
set -euo pipefail

lfc_src_pkg_name="org.lflang.lfc"
lfc_jar_snapshot_path="${lfc_src_pkg_name}/build/libs/${lfc_src_pkg_name}-*-SNAPSHOT-all.jar"
lfc_jar_release_path="lib/jars/${lfc_src_pkg_name}-*-SNAPSHOT-all.jar"

# Obtain path to the directory containing this script, even in presence of links.
bindir=`dirname "$(readlink -f "$0")"`
# Get to the base by going from "./lib/scripts" to ".".
base=`dirname $(dirname ${bindir})`

# Check if the sources are available.
function dir_exists {
    if [ -d "${1}" ]; then
        true
    else
        false
    fi
}

# Enter directory silently (without printing).
pushd () {
    command pushd "$@" > /dev/null
}

# Leave directory silently (without printing).
popd () {
    command popd "$@" > /dev/null
}

# Report fatal error and exit.
function fatal_error() {
    1>&2 echo -e "\e[1mlfc: \e[31mfatal error: \e[0m$1"
    exit 1
}

# Set the jar_path variable if a valid jar can be found and return true.
# If not valid jar can be found, return false.
function find_jar_path() {
    if src_exists; then
        jar_path_pattern="${base}/${lfc_jar_snapshot_path}"
    else
        jar_path_pattern="${base}/${lfc_jar_release_path}"
    fi

    #echo "Jar pattern: ${jar_path_pattern}"

    # Is there a file that matches our pattern?
    if ls ${jar_path_pattern} 1> /dev/null 2>&1; then
        # Yes. Determine the precise path of the jar. Take the newest version if
        # there are multiple jars.
        jar_path="$(ls  ${jar_path_pattern} | sort -V | tail -n1)"
        true
    else
        # No.
        false
    fi
}

# Lookup the JRE.
function lookup_jre() {
    if [[ $(type -p java) != "" ]]; then
        #echo found java executable in PATH
        _java=java
    elif [[ -n "$JAVA_HOME" ]] && [[ -x "$JAVA_HOME/bin/java" ]];  then
        #echo found java executable in JAVA_HOME     
        _java="$JAVA_HOME/bin/java"
    else
        fatal_error "JRE not found."
    fi
}

# Check whether the JRE version is high enough. Exit with an error if it is not.
function check_jre_version {
    lookup_jre
    if [[ "$_java" ]]; then
        semantic_version=$("$_java" -version 2>&1 | awk -F '"' '/version/ {print $2}')
        java_version=$(echo "$semantic_version" | awk -F. '{printf("%03d%03d",$1,$2);}')
        #echo version "$semantic_version"
        #echo version "$java_version"
        if [ $java_version -lt 011000 ]; then
            fatal_error "JRE $semantic_version found but 1.11 or greater is required."
        fi
    fi
}

# Run the found jar using the found JRE.
function run_jar_with_args {
    "${_java}" -jar "${jar_path}" "$@";
    exit $?
}

# Determine whether or not we are in a source tree.
function src_exists {
    #echo "Src dir: ${base}/${lfc_src_pkg_name}"
    if dir_exists "${base}/${lfc_src_pkg_name}"; then
        true
    else
        false
    fi
}
