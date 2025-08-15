#!/bin/bash

# Test script for the blueprint warehouse simulation
# This script builds and launches the warehouse simulation based on the architectural drawing

set -e  # Exit on any error

echo "🚀 Starting Blueprint Warehouse Simulation"
echo "=========================================="
echo ""
echo "This simulation includes:"
echo "  ✓ Accurate warehouse layout based on architectural drawing"
echo "  ✓ Rooms 101-FRYS, 102-KJØL, 104-TØRRVARE, 105-TØRRVARE"
echo "  ✓ Industrial racks in north-south orientation"
echo "  ✓ 100 pallets distributed throughout"
echo "  ✓ Functioning doors (C, D, F types)"
echo "  ✓ Clear navigation areas"
echo "  ✓ Forklift model"
echo ""

# Change to workspace root
cd /home/saidul/Desktop/fork_ws

# Build the package
echo "📦 Building the workspace..."
colcon build --packages-select harmonic_sim
if [ $? -ne 0 ]; then
    echo "❌ Build failed! Please check for errors."
    exit 1
fi

# Source the workspace
echo "🔧 Sourcing workspace..."
source install/setup.bash

# Check if required packages are available
echo "🔍 Checking dependencies..."
if ! ros2 pkg list | grep -q "gazebo_ros"; then
    echo "❌ Error: gazebo_ros package not found. Please install ros-humble-gazebo-ros-pkgs"
    exit 1
fi

echo "✅ All dependencies satisfied"
echo ""

# Launch the simulation
echo "🌍 Launching warehouse simulation..."
echo "Press Ctrl+C to stop the simulation"
echo ""

ros2 launch harmonic_sim blueprint_warehouse_detailed.launch.py

echo ""
echo "🛑 Simulation stopped"