#!/usr/bin/env python3
"""
Script to replace all inline rack definitions with model references
"""
import re

# Read the world file
with open('/home/saidul/Desktop/fork_ws/src/harmonic_sim/worlds/blueprint_warehouse_detailed.world', 'r') as f:
    content = f.read()

# Pattern to match inline rack definitions
rack_pattern = r'    <model name="(rack_\d+_\d+)">\s*<static>true</static>\s*<pose>([^<]+)</pose>\s*<link name="link">\s*<collision name="collision">\s*<geometry>\s*<box>\s*<size>[^<]+</size>\s*</box>\s*</geometry>\s*</collision>\s*<visual name="visual">\s*<geometry>\s*<box>\s*<size>[^<]+</size>\s*</box>\s*</geometry>\s*<material>\s*<ambient>[^<]+</ambient>\s*<diffuse>[^<]+</diffuse>\s*</material>\s*</visual>\s*</link>\s*</model>'

def replace_rack(match):
    rack_name = match.group(1)
    pose = match.group(2)
    
    return f'''    <include>
      <uri>model://warehouse_rack</uri>
      <pose>{pose}</pose>
      <name>{rack_name}</name>
    </include>'''

# Replace all inline rack definitions
new_content = re.sub(rack_pattern, replace_rack, content, flags=re.MULTILINE | re.DOTALL)

# Write back to file
with open('/home/saidul/Desktop/fork_ws/src/harmonic_sim/worlds/blueprint_warehouse_detailed.world', 'w') as f:
    f.write(new_content)

print("✅ All rack definitions updated to reference warehouse_rack model!")