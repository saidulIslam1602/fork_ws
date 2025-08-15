#!/bin/bash

# Clear any existing Gazebo processes
pkill -f gazebo 2>/dev/null
pkill -f gzserver 2>/dev/null
pkill -f gzclient 2>/dev/null

# Set proper display and Qt platform
export DISPLAY=:0
export QT_QPA_PLATFORM=xcb
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/install/harmonic_sim/share/harmonic_sim/models

# Navigate to workspace
cd /home/saidul/Desktop/fork_ws

# Source the environment
source install/setup.bash

echo "=== Starting Warehouse Simulation ==="
echo "World file: install/harmonic_sim/share/harmonic_sim/worlds/blueprint_warehouse_detailed.world"
echo "Checking if world file exists..."

if [ -f "install/harmonic_sim/share/harmonic_sim/worlds/blueprint_warehouse_detailed.world" ]; then
    echo "✓ World file found!"
    echo "Rack count verification:"
    echo "Room 101 racks: $(grep -c "rack_101_" install/harmonic_sim/share/harmonic_sim/worlds/blueprint_warehouse_detailed.world)"
    echo "Room 105 racks: $(grep -c "rack_105_" install/harmonic_sim/share/harmonic_sim/worlds/blueprint_warehouse_detailed.world)"
    echo ""
    echo "Starting Gazebo..."
    gazebo --verbose install/harmonic_sim/share/harmonic_sim/worlds/blueprint_warehouse_detailed.world
else
    echo "✗ World file not found!"
    exit 1
fi