#!/bin/bash

#============================================================================
# Description: 	Build the Lingua Franca compiler.
# Authors:		Marten Lohstroh, Mehrdad Niknami, Christian Menard
# Usage:		build-lfc [options] [[-r | --run] [lfc-args]]
#============================================================================

# Initialize the environment.
source "$(dirname "$(readlink -f "$0")")/init.sh" 

# Find the jar and check whether sources are present or not.
find_jar_path

if ! src_exists; then
    fatal_error "Cannot find sources."
fi

# Print message explaining the CLI args.
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

if [ $clean -eq 1 ]; then
    echo "Performing cleanup..."
    "${base}/gradlew" -p "${base}" clean;
fi

# Check if jar is missing or out-of-date compared to the sources
if [ ! -f "${lfc_jar_build_path}" ] || ! "${FIND}" "${lfbase}/src" -path "${lfbase}/test" -prune -o -type f -newer "${lfc_jar_build_path}" -exec false {} +; then
	1>&2 echo "Jar file is missing or out-of-date; running Gradle..."
	"${base}/gradlew" ${flags} -p "${base}" buildLfc
	touch -c -- "${jar_path}"  # Ensure the file timestamp is up-to-date even if the file didn't need to be updated
else
    echo "Already up-to-date."
fi

# Run lfc with the provided arguments.
if [[ "${run}" == "true" ]]; then
    check_jre_version
    echo "Running lfc..."
    run_jar_with_args "${args[@]}"
fi