#!/usr/bin/env python3
"""
Script to fully comment out all rack model blocks in the warehouse world file
"""

# Read the world file
world_file = '/home/saidul/Desktop/fork_ws/src/harmonic_sim/worlds/blueprint_warehouse_detailed.world'

with open(world_file, 'r') as f:
    lines = f.readlines()

print(f"Original file has {len(lines)} lines")

# Track which lines to comment out
comment_lines = []
inside_rack_model = False
rack_start_line = None

for i, line in enumerate(lines):
    # Check if we're starting a rack model
    if '<model name="rack_' in line:
        inside_rack_model = True
        rack_start_line = i
        print(f"Found rack model starting at line {i+1}: {line.strip()}")
    
    # If we're inside a rack model, mark this line for commenting
    if inside_rack_model:
        comment_lines.append(i)
    
    # Check if we're ending a rack model
    if inside_rack_model and '</model>' in line:
        inside_rack_model = False
        print(f"  -> Rack model ends at line {i+1}, total lines: {i - rack_start_line + 1}")

print(f"Total lines to comment out: {len(comment_lines)}")

# Comment out the marked lines
for line_idx in comment_lines:
    original_line = lines[line_idx]
    if original_line.strip() and not original_line.strip().startswith('<!--'):
        # Add comment markers around the content
        indentation = len(original_line) - len(original_line.lstrip())
        indent = original_line[:indentation]
        content = original_line[indentation:].rstrip()
        lines[line_idx] = f"{indent}<!-- {content} -->\n"

# Write the updated file
with open(world_file, 'w') as f:
    f.writelines(lines)

print(f"✅ All rack models fully commented out!")
print(f"New file has {len(lines)} lines")