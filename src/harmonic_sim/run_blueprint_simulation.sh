#!/bin/bash

# Blueprint Warehouse Simulation Quick Start Script
# This script launches the Gazebo simulation with the new warehouse layout based on your architectural blueprint

echo "=========================================="
echo "  Blueprint Warehouse Simulation"
echo "=========================================="
echo ""
echo "Starting Gazebo simulation with the new warehouse layout..."
echo "Based on your architectural blueprint drawing."
echo ""

# Set up ROS 2 environment (adjust if needed)
source /opt/ros/humble/setup.bash

# Check if the workspace is built
if [ ! -d "install" ]; then
    echo "Building the workspace first..."
    colcon build --packages-select harmonic_sim
    if [ $? -ne 0 ]; then
        echo "Build failed! Please check for errors."
        exit 1
    fi
fi

# Source the workspace
source install/setup.bash

# Check if the world file exists
WORLD_FILE="worlds/blueprint_warehouse.world"
if [ ! -f "$WORLD_FILE" ]; then
    echo "Error: Blueprint warehouse world file not found at $WORLD_FILE"
    echo "Please make sure the file exists and try again."
    exit 1
fi

echo "World file found: $WORLD_FILE"
echo ""

# Launch arguments
USE_GUI="true"
VERBOSE="false"

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --headless)
            USE_GUI="false"
            echo "Running in headless mode (no GUI)"
            shift
            ;;
        --verbose)
            VERBOSE="true"
            echo "Verbose output enabled"
            shift
            ;;
        --help|-h)
            echo "Usage: $0 [options]"
            echo ""
            echo "Options:"
            echo "  --headless    Run Gazebo without GUI"
            echo "  --verbose     Enable verbose output"
            echo "  --help, -h    Show this help message"
            echo ""
            echo "The simulation includes:"
            echo "  - Large warehouse with multiple storage aisles"
            echo "  - Office areas and administrative rooms"
            echo "  - Loading dock areas"
            echo "  - Technical rooms"
            echo "  - Realistic lighting and equipment"
            echo "  - Forklift robot for navigation testing"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

echo "Launching blueprint warehouse simulation..."
echo "- GUI: $USE_GUI"
echo "- Verbose: $VERBOSE"
echo ""
echo "Features of this simulation:"
echo "  ✓ Large-scale warehouse (120m x 80m)"
echo "  ✓ 5 storage aisles with realistic rack systems"
echo "  ✓ Office areas and administrative rooms"
echo "  ✓ Loading dock facilities"
echo "  ✓ Technical equipment rooms"
echo "  ✓ Proper industrial lighting"
echo "  ✓ Safety markings and equipment"
echo ""

# Launch the simulation
if [ "$USE_GUI" = "false" ]; then
    ros2 launch harmonic_sim blueprint_warehouse.launch.py headless:=true verbose:=$VERBOSE
else
    ros2 launch harmonic_sim blueprint_warehouse.launch.py headless:=false verbose:=$VERBOSE
fi

echo ""
echo "Simulation ended."
echo "Thank you for using the Blueprint Warehouse Simulation!" 