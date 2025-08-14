#!/bin/bash

echo "🚀 Testing Harmonic Forklift Simulation - Step by Step"

# Navigate to workspace
cd ~/Desktop/fork_ws

# Build and source
echo "📦 Building project..."
colcon build
echo "🔧 Sourcing environment..."
source install/setup.bash

# Set model path
export GAZEBO_MODEL_PATH=/home/saidul/Desktop/fork_ws/src/harmonic_sim/models:$GAZEBO_MODEL_PATH
echo "🗂️  Model path set: $GAZEBO_MODEL_PATH"

# Test 1: Check if package is found
echo "🔍 Testing package availability..."
ros2 pkg list | grep harmonic_sim
if [ $? -eq 0 ]; then
    echo "✅ harmonic_sim package found!"
else
    echo "❌ Package not found!"
    exit 1
fi

# Test 2: Check world file exists
echo "🔍 Checking world file..."
if [ -f "src/harmonic_sim/worlds/harmonic.world" ]; then
    echo "✅ World file found!"
else
    echo "❌ World file not found!"
    exit 1
fi

# Test 3: Start basic Gazebo
echo "🌍 Starting Gazebo..."
echo "You should see Gazebo open with a forklift robot"
echo "Press Ctrl+C in this terminal to stop"

gazebo --verbose src/harmonic_sim/worlds/harmonic.world 