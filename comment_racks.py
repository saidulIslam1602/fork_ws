#!/usr/bin/env python3
"""
Script to comment out all rack models in the warehouse world file
"""

import re

# Read the world file
world_file = '/home/saidul/Desktop/fork_ws/src/harmonic_sim/worlds/blueprint_warehouse_detailed.world'

with open(world_file, 'r') as f:
    content = f.read()

print(f"Original file size: {len(content.splitlines())} lines")

# Find and comment out all rack models
# Look for models that start with "rack_" 
rack_pattern = r'(<model name="rack_.*?</model>)'

def comment_rack_model(match):
    model_content = match.group(1)
    # Split into lines and add <!-- --> comments
    lines = model_content.split('\n')
    commented_lines = []
    for line in lines:
        if line.strip():
            commented_lines.append('    <!-- ' + line.strip() + ' -->')
        else:
            commented_lines.append(line)
    return '\n'.join(commented_lines)

# Replace all rack models with commented versions
new_content = re.sub(rack_pattern, comment_rack_model, content, flags=re.DOTALL)

print(f"Found and commented out rack models")

# Write the updated file
with open(world_file, 'w') as f:
    f.write(new_content)

print(f"✅ All rack models commented out!")
print(f"New file size: {len(new_content.splitlines())} lines")