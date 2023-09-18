#!/bin/bash
timeout=300 # 5min timeout is hopefully enough
verbose=false

# Function to recursively find all folders containing the top-level CMakeLists.txt
overall_success=true
num_successes=0
num_failures=0
failed_tests=""

# Skip
skip=("FileReader" "FilePkgReader" "Tracing" "ThreadedThreaded")

find_kconfig_folders() {
    if [ -f "$folder/CMakeLists.txt" ]; then
        echo "-----------------------------------------------------------------------------------------------------------"
        test_name=$(basename $folder)

        if [[ " ${skip[*]} " == *" $test_name "* ]]; then
            echo "Skipping: $test_name"
        else
            echo "Running: $test_name"
            if run_qemu_zephyr_test "$folder"; then
                echo "Test $test_name successful"
                let "num_successes+=1"
            else
                echo "Test $test_name failed"
                let "num_failures+=1"
                failed_tests+="$test_name, "
                overall_success=false
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

run_native_zephyr_test() {
    return_val=0
    pushd $1/build

    rm -f res.text

    timeout 60s make run | tee res.txt
    result=$?

    if [ $result -eq 0 ]; then
        echo "Command completed within the timeout."
        return_val=0
    else
        echo "Command terminated or timed out."
        echo "Test output:"
        echo "----------------------------------------------------------------"
        cat res.txt
        echo "----------------------------------------------------------------"
        return_val=1
    fi

    popd
    return "$return_val"




}

# Run Zephyr test until either: Timeout or finds match in output
# https://www.unix.com/shell-programming-and-scripting/171401-kill-process-if-grep-match-found.html
run_qemu_zephyr_test() {
    success=false
    pushd $1/build

    rm -f /tmp/procfifo
    rm -f res.text
    mkfifo /tmp/procfifo

    make run | tee res.txt >  /tmp/procfifo &
    PID=$!
    SECONDS=0
    while IFS= read -t $timeout line
    do
        if [ "$verbose" = "true" ]; then  echo $line; fi

        if echo "$line" | grep -q 'FATAL ERROR';
        then
            echo "Matched on ERROR"
            echo $line
            success=false
            pkill -P $$
            break
        fi

        if echo "$line" | grep -q '^exit';
        then
                echo "Matched on exit"
                success=true
                pkill -P $$
                break
        fi

        if (($SECONDS > $timeout)) ; then
            echo "Timeout without freeze"
            success=false
            break
        fi
    done < /tmp/procfifo

    return_val=0
    if [ "$success" = false ]; then
        echo "General Timeout"
        pkill -P $$
        echo "Test output:"
        echo "----------------------------------------------------------------"
        cat res.txt
        echo "----------------------------------------------------------------"
        return_val=1
    fi

    rm -f /tmp/procfifo
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

if [ "$overall_success" = false ]; then
    echo "Results: FAIL"
else
    echo "Results: PASS"
fi
echo "Number of passes: $num_successes"
echo "Number of fails: $num_failures"
echo "Skipped tests: ${skip[@]}"

if [ "$overall_success" = false ]; then
    echo "Failed tests: $failed_tests"
    exit 1
fi

exit 0
