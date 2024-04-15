#!/bin/bash

compile_package() {
    local component_name="$1"
    
    if [ -d "$component_name" ]; then
      cd $component_name &&
        colcon build --event-handlers console_direct+ --symlink-install &&
        cd ..
    else
        echo "Unable to find $component_name"
        exit 1
    fi
}

compile_components() {
    local components=("$@")

    for component in "${components[@]}"; do
        compile_package "$component"
    done
}

components=(
    "Navigation_System"
    "Robot_Reader"
    "Robot_Writer"
    "Gazebo_Rumbo"
    "Battery"
)

git submodule update
compile_components "${components[@]}"
