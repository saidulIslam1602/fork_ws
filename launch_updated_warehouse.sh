#!/bin/bash

echo "🏭 WAREHOUSE SIMULATION LAUNCHER 🏭"
echo "==================================="

# Clean up any existing processes
echo "🧹 Cleaning up existing processes..."
pkill -f gazebo 2>/dev/null
pkill -f gzserver 2>/dev/null  
pkill -f gzclient 2>/dev/null
pkill -f ros2 2>/dev/null
sleep 2

# Set proper environment
export DISPLAY=:0
export QT_QPA_PLATFORM=xcb
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/install/harmonic_sim/share/harmonic_sim/models

# Navigate to workspace
cd /home/saidul/Desktop/fork_ws

# Build and source
echo "🔨 Building workspace..."
colcon build --packages-select harmonic_sim --quiet

echo "⚙️  Sourcing environment..."
source install/setup.bash

# Verify world file
WORLD_FILE="install/harmonic_sim/share/harmonic_sim/worlds/blueprint_warehouse_detailed.world"
if [ -f "$WORLD_FILE" ]; then
    echo "✅ World file verified!"
    echo "📊 Rack Statistics:"
    echo "   • Room 101-FRYS: $(grep -c "rack_101_" $WORLD_FILE) racks"
    echo "   • Room 102-KJØL: $(grep -c "rack_102_" $WORLD_FILE) racks" 
    echo "   • Room 104-TØRRVARE: $(grep -c "rack_104_" $WORLD_FILE) racks"
    echo "   • Room 105-TØRRVARE: $(grep -c "rack_105_" $WORLD_FILE) racks"
    echo "   • Total: $(($(grep -c "rack_101_" $WORLD_FILE) + $(grep -c "rack_102_" $WORLD_FILE) + $(grep -c "rack_104_" $WORLD_FILE) + $(grep -c "rack_105_" $WORLD_FILE))) racks"
    echo ""
    echo "🚀 Starting warehouse simulation..."
    echo "   World: $WORLD_FILE"
    echo ""
    
    # Launch the simulation
    ros2 launch harmonic_sim blueprint_warehouse_detailed.launch.py
else
    echo "❌ World file not found: $WORLD_FILE"
    exit 1
fi