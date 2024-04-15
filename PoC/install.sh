#!/bin/bash

LOG_PREFIX="[INSTALLER]"

log() {
    echo "$LOG_PREFIX $@"
}

compile_package() {
    local component_name="$1"
    
    log "Build $component_name."
    
    if [ -d "$component_name" ]; then
      cd $component_name
	log "Clearing build directories..."
	rm -rf build/ log/ install/
	rosdep update
	rosdep install --from-paths src --ignore-src -r -y -i --os="$OS"
	log "Build..."
	colcon build --symlink-install
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

# Update submodules
log "Updating submodules..."
git submodule update --init --recursive
log "Submodules updated."

# Initialize rosdep only if it hasn't been initialized before
if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
    log "Initializing rosdep..."
    sudo rosdep init
    log "Rosdep initialized."
fi

# Declare the OS used
OS="ubuntu:jammy"
log "Operating system: $OS"

compile_components "${components[@]}"