#!/bin/bash

# Skip
skip=("FileReader" "FilePkgReader")

find_kconfig_folders() {
    if [ -f "$folder/CMakeLists.txt" ]; then
        echo "-----------------------------------------------------------------------------------------------------------"
        test_name=$(basename $folder)

        if [[ " ${skip[*]} " == *" $test_name "* ]]; then
            echo "Skipping: $test_name"
        else
            echo "Running: $test_name"
            run_qemu_zephyr_test $folder
            if [ "$?" -eq 0 ]; then
                echo "Test passed"
                let "num_successes+=1"
            else
                echo "Test failed."
                exit 1
            fi
        fi
      return
    fi
    for folder in "$1"/*; do
        if [ -d "$folder" ]; then
            find_kconfig_folders "$folder"
        fi
    done
}

# Run Zephyr test until either: Timeout or finds match in output using expect
# https://spin.atomicobject.com/monitoring-stdout-with-a-timeout/
run_qemu_zephyr_test() {
    success=false
    pushd $1/build
    res="_res.txt"
    # Run test program in background and pipe results to a file.
    make run > $res &
    if [ $? -ne 0 ]; then
        echo "ERROR: make run failed."
        exit 1
    fi
    pid=$!
    return_val=2
    wait_count=0
    timeout=120
    # Parse the file and match on known outputs. With timeout.
    while [ "$wait_count" -le "$timeout" ]; do
        sleep 1
        if grep --quiet 'FATAL ERROR' "$res"
        then
            cat $res
            echo "-----------------------------------------------------------------------------------------------------------"
            echo "ERROR: Found 'FATAL ERROR'"
            return_val=1
            break
        fi
        if grep --quiet '^exit' "$res"
        then
            cat $res
            echo "-----------------------------------------------------------------------------------------------------------"
            echo "SUCCESS: Found 'exit'"
            return_val=0
            break
        fi

        ((wait_count++))
    done
    kill $pid
    if [ $? -ne 0 ]; then
        echo "ERROR: Could not kill qemu process"
        exit 1
    fi

    if [ "$return_val" -eq 2 ]; then
        cat $res
        echo "-----------------------------------------------------------------------------------------------------------"
        echo "ERROR: Timed out"
    fi
    popd
    return "$return_val"
}

# Change directory to the specified path
cd "$1"

# Call the find_kconfig_folders function with the current directory as the starting point
find_kconfig_folders "."

# Change back to the original directory
cd -

# Print report
echo "================================================================================================================"
echo "Tests passed!"
echo "Number of passes: $num_successes"
echo "Skipped tests: ${skip[@]}"

if [ "$overall_success" = false ]; then
    echo "Failed tests: $failed_tests"
    exit 1
fi

exit 0
