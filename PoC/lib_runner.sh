#!/bin/bash

source_local_setup() {
    local setup_file="$1"
    local component_name="$2"
    
    if [ -f "$setup_file" ]; then
        . "$setup_file"
    else
        echo "You need to compile $component_name"
        exit 1
    fi
}

clean_storage_folder() {
    echo "Cleaning Storage folder"
    find "$(dirname "$0")/../Storage/from_ap" -depth -type f \( -name '*.txt' -o -name '*.lock' \) -delete -o -path "$(dirname "$0")/Storage/from_ap" -prune -type f \( -name '*.txt' -o -name '*.lock' \) -delete
    find "$(dirname "$0")/../Storage/from_robot" -depth -type f \( -name '*.txt' -o -name '*.lock' \) -delete -o -path "$(dirname "$0")/Storage/from_robot" -prune -type f \( -name '*.txt' -o -name '*.lock' \) -delete
    find "$(dirname "$0")/../Storage/from_user" -depth -type f \( -name '*.txt' -o -name '*.lock' \) -delete -o -path "$(dirname "$0")/Storage/from_user" -prune -type f \( -name '*.txt' -o -name '*.lock' \) -delete
    echo "Cleaned Storage folder"
}

initialize_components() {
    local components=("$@")

    for component in "${components[@]}"; do
        source_local_setup "$(dirname "$0")/$component/install/local_setup.bash" "$component"
    done
}
