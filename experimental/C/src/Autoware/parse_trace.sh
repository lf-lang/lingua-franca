# This script will generate 4 timelines for the important components in Autoware
# based on the stored output of Autoware (e.g., './bin/Autoware > trace' and 'parse_trace.sh trace').
grep -nre "Fusion" $1 | awk '{print $5}' | awk '{if (NR==1) {first_row = $1} printf "%s", ($1 - first_row)/1000000; printf "\n";}' > fusion_physical_timeline # Physical time
grep -nre "Actuator" $1 | awk '{print $5}' | awk '{if (NR==1) {first_row = $1} printf "%s", ($1 - first_row)/1000000; printf "\n";}' > actuator_physical_timeline # Physical time
grep -nre "Fusion" $1 | awk '{print $7}' | awk '{if (NR==1) {first_row = $1} printf "%s", ($1 - first_row)/1000000; printf "\n";}' > fusion_logical_timeline # Logical time
grep -nre "Actuator" $1 | awk '{print $7}' | awk '{if (NR==1) {first_row = $1} printf "%s", ($1 - first_row)/1000000; printf "\n";}' > actutator_logical_timeline # Logical time
