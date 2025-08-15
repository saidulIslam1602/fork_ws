#!/usr/bin/env python3
"""
Script to remove all Room 106 components from the warehouse world file
"""

import re

# Read the world file
world_file = '/home/saidul/Desktop/fork_ws/src/harmonic_sim/worlds/blueprint_warehouse_detailed.world'

with open(world_file, 'r') as f:
    lines = f.readlines()

print(f"Original file has {len(lines)} lines")

# Find all Room 106 components to remove
room_106_patterns = [
    'room_106_south_wall',
    'room_106_east_wall', 
    'room_106_west_wall',
    'room_106_floor',
    'room_106_test_marker',
    'room_106_lager_tank_1',
    'room_106_lager_tank_2',
    'sign_106_torg_inn'
]

# Track which sections to remove
sections_to_remove = []

for i, line in enumerate(lines):
    for pattern in room_106_patterns:
        if f'<model name="{pattern}">' in line:
            # Find the start of this model
            model_start = i
            
            # Find the end of this model (next </model>)
            model_end = None
            for j in range(i + 1, len(lines)):
                if '</model>' in lines[j]:
                    model_end = j
                    break
            
            if model_end:
                sections_to_remove.append((model_start, model_end))
                print(f"Found {pattern} from line {model_start+1} to {model_end+1}")

# Also remove the Room 106 comment sections
for i, line in enumerate(lines):
    if 'Room 106' in line and ('<!--' in line or '===' in line):
        # Find associated comment block
        if '<!--' in line:
            # Single line comment
            sections_to_remove.append((i, i))
            print(f"Found Room 106 comment at line {i+1}")
        elif '===' in line:
            # Section divider
            sections_to_remove.append((i, i))
            print(f"Found Room 106 section divider at line {i+1}")

# Sort sections by start line (reverse order to remove from end first)
sections_to_remove.sort(key=lambda x: x[0], reverse=True)

# Remove the sections
lines_removed = 0
for start, end in sections_to_remove:
    print(f"Removing lines {start+1}-{end+1}")
    del lines[start:end+1]
    lines_removed += (end - start + 1)

print(f"Removed {lines_removed} lines")
print(f"New file has {len(lines)} lines")

# Write the updated file
with open(world_file, 'w') as f:
    f.writelines(lines)

print("✅ Room 106 removal complete!")