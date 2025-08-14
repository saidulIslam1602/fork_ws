#!/bin/bash

# Test script for the detailed blueprint warehouse simulation
# This script sets up the environment and launches the warehouse simulation

set -e  # Exit on any error

echo "🚀 Starting Blueprint Warehouse Simulation Test"
echo "=============================================="

# Check if we're in the right directory
if [ ! -f "package.xml" ]; then
    echo "❌ Error: package.xml not found. Please run this script from the harmonic_sim package root."
    exit 1
fi

# Source the workspace
echo "📦 Sourcing workspace..."
if [ -f "../install/setup.bash" ]; then
    source ../install/setup.bash
    echo "✅ Workspace sourced successfully"
elif [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo "✅ Workspace sourced successfully"
else
    echo "❌ Error: Could not find setup.bash. Please build the workspace first."
    exit 1
fi

# Check if the world file exists
WORLD_FILE="worlds/blueprint_warehouse_detailed.world"
if [ ! -f "$WORLD_FILE" ]; then
    echo "❌ Error: World file $WORLD_FILE not found!"
    exit 1
fi

echo "🌍 World file found: $WORLD_FILE"

# Check if the launch file exists
LAUNCH_FILE="launch/blueprint_warehouse_detailed.launch.py"
if [ ! -f "$LAUNCH_FILE" ]; then
    echo "❌ Error: Launch file $LAUNCH_FILE not found!"
    exit 1
fi

echo "🚀 Launch file found: $LAUNCH_FILE"

# Check if required packages are available
echo "🔍 Checking required packages..."

# Check for gazebo_ros
if ! ros2 pkg list | grep -q "gazebo_ros"; then
    echo "❌ Error: gazebo_ros package not found. Please install it first."
    exit 1
fi

# Check for nav2_bringup
if ! ros2 pkg list | grep -q "nav2_bringup"; then
    echo "❌ Error: nav2_bringup package not found. Please install it first."
    exit 1
fi

# Check for slam_toolbox
if ! ros2 pkg list | grep -q "slam_toolbox"; then
    echo "❌ Error: slam_toolbox package not found. Please install it first."
    exit 1
fi

echo "✅ All required packages are available"

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "🧹 Cleaning up..."
    pkill -f "gazebo" || true
    pkill -f "rviz2" || true
    pkill -f "ros2" || true
    echo "✅ Cleanup completed"
}

# Set trap to cleanup on script exit
trap cleanup EXIT

echo ""
echo "🎯 Launching the detailed warehouse simulation..."
echo "   - World: blueprint_warehouse_detailed.world"
echo "   - Forklift will spawn at position (0, -50, 1)"
echo "   - Use RViz2 for visualization"
echo "   - Press Ctrl+C to stop the simulation"
echo ""

# Launch the simulation
ros2 launch harmonic_sim blueprint_warehouse_detailed.launch.py

echo ""
echo "🏁 Simulation completed" 