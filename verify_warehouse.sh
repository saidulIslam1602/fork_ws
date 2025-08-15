#!/bin/bash

echo "🔍 WAREHOUSE WORLD VERIFICATION 🔍"
echo "=================================="

cd /home/saidul/Desktop/fork_ws

# Build first to ensure latest changes
colcon build --packages-select harmonic_sim --quiet
source install/setup.bash

WORLD_FILE="install/harmonic_sim/share/harmonic_sim/worlds/blueprint_warehouse_detailed.world"

if [ -f "$WORLD_FILE" ]; then
    echo "✅ World file exists: $WORLD_FILE"
    echo ""
    
    # Check file size and modification time
    echo "📄 File Info:"
    ls -lh "$WORLD_FILE"
    echo ""
    
    # Verify rack counts
    echo "🏗️  Rack Distribution:"
    ROOM_101=$(grep -c "rack_101_" "$WORLD_FILE")
    ROOM_102=$(grep -c "rack_102_" "$WORLD_FILE") 
    ROOM_104=$(grep -c "rack_104_" "$WORLD_FILE")
    ROOM_105=$(grep -c "rack_105_" "$WORLD_FILE")
    TOTAL=$((ROOM_101 + ROOM_102 + ROOM_104 + ROOM_105))
    
    echo "   • Room 101-FRYS:     $ROOM_101 racks"
    echo "   • Room 102-KJØL:     $ROOM_102 racks"
    echo "   • Room 104-TØRRVARE: $ROOM_104 racks"  
    echo "   • Room 105-TØRRVARE: $ROOM_105 racks"
    echo "   • TOTAL:             $TOTAL racks"
    echo ""
    
    # Check for door segments
    echo "🚪 Door Configuration:"
    NORTH_WALLS=$(grep -c "north_wall_[1-6]" "$WORLD_FILE")
    SOUTH_WALLS=$(grep -c "south_wall_[1-6]" "$WORLD_FILE")
    DIVIDERS=$(grep -c "divider.*north\|divider.*south" "$WORLD_FILE")
    
    echo "   • North wall segments: $NORTH_WALLS"
    echo "   • South wall segments: $SOUTH_WALLS" 
    echo "   • Room dividers:       $DIVIDERS"
    echo ""
    
    # Check Room 104 size
    echo "📐 Room 104 Size Verification:"
    ROOM_104_SIZE=$(grep -A 5 "room_104_floor" "$WORLD_FILE" | grep "size" | head -1)
    echo "   • Floor size: $ROOM_104_SIZE"
    echo ""
    
    if [ $TOTAL -ge 40 ]; then
        echo "✅ SUCCESS: World file is properly updated with $TOTAL racks!"
    else
        echo "⚠️  WARNING: Expected 40+ racks, found only $TOTAL"
    fi
    
else
    echo "❌ World file not found!"
    exit 1
fi