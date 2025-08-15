#!/usr/bin/env python3
"""
Update warehouse rack layout for proper forklift navigation.
Creates main aisles aligned with doors and reduces rack count for navigation.
"""

import re

def update_warehouse_navigation():
    file_path = 'src/harmonic_sim/worlds/blueprint_warehouse_detailed.world'
    
    # New rack configuration with navigation aisles
    # Format: room_name: [(x_pos, y_north, y_south), ...]
    new_rack_positions = {
        '101': [  # Room 101 (FRYS): 4 racks with aisle at x=-40.75m
            (-49, 20, -20),  # West side
            (-56, 20, -20),
            (-32, 20, -20),  # East side  
            (-25, 20, -20),
        ],
        '102': [  # Room 102 (KJØL): 2 racks with aisle at x=-17.85m
            (-24, 25, -25),  # West side
            (-12, 25, -25),  # East side
        ],
        '104': [  # Room 104 (TØRRVARE): 4 racks with aisle at x=3.4m
            (-4, 25, -25),   # West side
            (-10, 25, -25),
            (11, 25, -25),   # East side
            (17, 25, -25),
        ],
        '105': [  # Room 105 (TØRRVARE): 6 racks with aisle at x=34.7m
            (22, 30, -30),   # West side
            (16, 30, -30),
            (10, 30, -30),
            (47, 30, -30),   # East side
            (53, 30, -30),
            (59, 30, -30),
        ]
    }

    print("🔄 Updating warehouse for navigation layout...")
    
    # Read the current file
    with open(file_path, 'r') as f:
        content = f.read()
    
    # Remove all existing racks (they're embedded inline now)
    # Find all rack model blocks and remove them
    rack_pattern = r'(\s*<model name="rack_\d{3}_\d+">\s*<static>true</static>\s*<pose>.*?</model>)'
    
    # Count existing racks before removal
    existing_racks = re.findall(r'<model name="(rack_\d{3}_\d+)">', content)
    print(f"Found {len(existing_racks)} existing racks to replace")
    
    # Remove all existing rack definitions
    content = re.sub(rack_pattern, '', content, flags=re.DOTALL)
    
    # Read the warehouse_rack model template
    with open('src/harmonic_sim/models/warehouse_rack/model.sdf', 'r') as f:
        rack_sdf_content = f.read()
    
    # Extract the inner content of the model (remove outer <model> tags)
    rack_inner_match = re.search(r"<model[^>]*>(.*)</model>", rack_sdf_content, re.DOTALL)
    if rack_inner_match:
        rack_sdf_inner = rack_inner_match.group(1).strip()
    else:
        raise ValueError("Could not find <model> tags in warehouse_rack/model.sdf")
    
    # Find where to insert new racks (after the room floors, before any existing models)
    insert_point = content.find('    <!-- =================================================================== -->')
    if insert_point == -1:
        insert_point = content.find('    <!-- WAREHOUSE STRUCTURE')
    
    # Build new rack definitions
    new_racks_text = "\n    <!-- ================================================================= -->\n"
    new_racks_text += "    <!-- NAVIGATION-OPTIMIZED RACK LAYOUT                               -->\n"
    new_racks_text += "    <!-- Main aisles aligned with doors for forklift navigation         -->\n"
    new_racks_text += "    <!-- ================================================================= -->\n\n"
    
    total_racks = 0
    for room, positions in new_rack_positions.items():
        new_racks_text += f"    <!-- Room {room} racks -->\n"
        
        for i, (x, y_north, y_south) in enumerate(positions, 1):
            # North rack
            rack_name = f"rack_{room}_{i}_north"
            new_racks_text += f"    <model name=\"{rack_name}\">\n"
            new_racks_text += f"      <static>true</static>\n"
            new_racks_text += f"      <pose>{x} {y_north} 0 0 0 0</pose>\n"
            new_racks_text += f"      {rack_sdf_inner}\n"
            new_racks_text += f"    </model>\n\n"
            total_racks += 1
            
            # South rack (mirror)
            rack_name = f"rack_{room}_{i}_south"
            new_racks_text += f"    <model name=\"{rack_name}\">\n"
            new_racks_text += f"      <static>true</static>\n"
            new_racks_text += f"      <pose>{x} {y_south} 0 0 0 0</pose>\n"
            new_racks_text += f"      {rack_sdf_inner}\n"
            new_racks_text += f"    </model>\n\n"
            total_racks += 1
        
        new_racks_text += "\n"
    
    # Insert the new racks
    content = content[:insert_point] + new_racks_text + content[insert_point:]
    
    # Write the updated file
    with open(file_path, 'w') as f:
        f.write(content)
    
    print(f"✅ Navigation layout updated!")
    print(f"   • Removed: {len(existing_racks)} old racks")
    print(f"   • Added: {total_racks} navigation-optimized racks")
    print(f"   • Room 101: {len(new_rack_positions['101'])*2} racks")
    print(f"   • Room 102: {len(new_rack_positions['102'])*2} racks") 
    print(f"   • Room 104: {len(new_rack_positions['104'])*2} racks")
    print(f"   • Room 105: {len(new_rack_positions['105'])*2} racks")
    print(f"🚚 Main aisles created at each door for forklift navigation!")

if __name__ == "__main__":
    update_warehouse_navigation()