#!/bin/bash

#============================================================================
# Description:      Run the Lingua Franca compiler.
# Authors:          Marten Lohstroh
#                   Christian Menard
# Usage:            Usage: lfc [options] files...
#============================================================================

# Initialize the environment.
source "$(dirname "$(readlink -f "$0")")/init.sh" 

# Find the jar; report error if it was not found.
if ! find_jar_path; then
    fatal_error "Cannot find lfc jar."
fi

# Make sure the correct JRE is available.
check_jre_version

# Lunch the compiler.
run_jar_with_args "$@"
