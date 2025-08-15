#!/bin/bash

echo "🚀 **LAUNCHING GAZEBO WITH NEW ROOMS** 🚀"
echo ""

# Navigate to workspace
cd /home/saidul/Desktop/fork_ws

# Kill any existing processes
echo "Cleaning existing processes..."
pkill -f gazebo 2>/dev/null
pkill -f gzserver 2>/dev/null
pkill -f gzclient 2>/dev/null
sleep 2

# Source environment
echo "Sourcing ROS2 environment..."
source /opt/ros/humble/setup.bash
source install/setup.bash

# Set display variables
export DISPLAY=:0
export QT_QPA_PLATFORM=xcb

# Verify package exists
echo "Checking package..."
if ros2 pkg list | grep -q harmonic_sim; then
    echo "✅ harmonic_sim package found"
else
    echo "❌ harmonic_sim package not found"
    exit 1
fi

# Launch with verbose output
echo ""
echo "🎯 **WHAT TO LOOK FOR:**"
echo "   🟥 Giant RED cube (Room 106) - 25x25x25m"
echo "   🟪 Giant PURPLE cube (Room 107) - 30x30x30m"
echo "   🏢 No racks (all commented out)"
echo "   📍 New rooms at south wall"
echo ""

echo "Launching..."
ros2 launch harmonic_sim blueprint_warehouse_detailed.launch.py