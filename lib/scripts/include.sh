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

[[ $(type -t fatal_error) == function ]] || echo "Warning: function 'fatal_error' is not defined."

[[ "${base}" ]] || fatal_error "Unable to determine absolute path to $0."

# Paths (relative to ${base}), which is assumed to have been set.
src_pkg_name="org.lflang"
src_pkg_path="${base}/${src_pkg_name}"
lfc_src_pkg_name="${src_pkg_name}.lfc"
lfc_src_pkg_path="${base}/${lfc_src_pkg_name}"
lfc_jar_build_path_pattern="${lfc_src_pkg_name}/build/libs/${lfc_src_pkg_name}-*-all.jar"
lfc_jar_release_path_pattern="lib/jars/${lfc_src_pkg_name}-*-all.jar"

# Enter directory silently (without printing).
pushd() {
    command pushd "$@" > /dev/null
}

# Leave directory silently (without printing).
popd() {
    command popd "$@" > /dev/null
}

# Check whether the first argument is a directory, and return it if true.
function is_dir() {
    if [ -d "${1}" ]; then
        echo "${1}"
    else
        return
    fi
}

# Check whether the source directory exists, and return it if true.
function get_src_dir() {
    echo `is_dir "${base}/${lfc_src_pkg_name}"`
}

# If it exists, return a path to the Lingua Franca jar.
function get_jar_path() {
    if [ "$(get_src_dir)" ]; then
        jar_path_pattern="${base}/${lfc_jar_build_path_pattern}"
    else
        jar_path_pattern="${base}/${lfc_jar_release_path_pattern}"
    fi
    #echo Jar path pattern: "${jar_path_pattern}"
    # Is there a file that matches our pattern? If so, return it.
    if ls ${jar_path_pattern} 1> /dev/null 2>&1; then
        # Yes. Determine the precise path of the jar. 
        # Take the newest version if there are multiple jars.
        echo "$(ls ${jar_path_pattern} | sort -V | tail -n1)"
    fi
}

# Lookup the JRE.
function lookup_jre() {
    if [[ $(type -p java) != "" ]]; then
        #echo Found java executable in PATH
        echo "java"
    elif [[ -n "$JAVA_HOME" ]] && [[ -x "$JAVA_HOME/bin/java" ]];  then
        #echo Found java executable in JAVA_HOME     
        echo "$JAVA_HOME/bin/java"
    else
        fatal_error "JRE not found."
    fi
}

# Check whether the JRE version is high enough. Exit with an error if it is not.
function get_java_cmd {
    java_cmd="$(lookup_jre)"
    if [[ "${java_cmd}" ]]; then
        semantic_version=$("$java_cmd" -version 2>&1 | awk -F '"' '/version/ {print $2}')
        java_version=$(echo "$semantic_version" | awk -F. '{printf("%03d%03d",$1,$2);}')
        #echo Semantic version: "$semantic_version"
        #echo Java version: "$java_version"
        if [ $java_version -lt 011000 ]; then
            fatal_error "JRE $semantic_version found but 1.11 or greater is required."
        fi
        echo ${java_cmd}
    fi
}

# Find the jar and JRE, run the jar with the provided arguments, and exit.
function run_lfc_with_args {
    # Find the jar; report error if it was not found.
    jar_path="$(get_jar_path)"

    if [[ ! "${jar_path}" ]]; then
        fatal_error "Cannot find the Lingua Franca jar."
    fi

    # Launch the compiler.
    "$(get_java_cmd)" -jar "${jar_path}" "$@";
    exit $?
}
