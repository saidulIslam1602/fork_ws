#!/bin/bash

# Quick Start Script for Harmonic Forklift Simulation
echo "🚀 Starting Harmonic Forklift Simulation..."

# Navigate to workspace
cd ~/Desktop/fork_ws

# Build the project
echo "📦 Building project..."
colcon build

# Source the environment
echo "🔧 Setting up environment..."
source install/setup.bash

# Set Gazebo model path
export GAZEBO_MODEL_PATH=/home/saidul/Desktop/fork_ws/src/harmonic_sim/models:$GAZEBO_MODEL_PATH

# Launch Gazebo with the world
echo "🌍 Launching Gazebo simulation..."
gazebo --verbose src/harmonic_sim/worlds/harmonic.world

echo "✅ Simulation started successfully!" 