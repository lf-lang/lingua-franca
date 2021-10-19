#!/bin/bash

#============================================================================
# Description: 	    Run the lfc compiler.
# Authors:          Marten Lohstroh
#                   Christian Menard
# Usage:            Usage: lfc [options] files...
#============================================================================

# Initialize the environment.
source "$(dirname "$(readlink -f "$0")")/init.sh" 

# Find the jar.
find_jarpath

# Check if a jar was found.
if [[ "${jar_found}" == "false" ]]; then
    fatal_error "Cannot find lfc jar"
fi

check_jre_version

# run the command
run_jar_with_args "$@"