#!/usr/bin/env python3
"""
Fix missing racks by embedding the industrial rack design directly in the world file
"""
import re

# Read the world file
with open('/home/saidul/Desktop/fork_ws/src/harmonic_sim/worlds/blueprint_warehouse_detailed.world', 'r') as f:
    content = f.read()

# Industrial rack model definition (inline)
industrial_rack_template = '''    <model name="{name}">
      <static>true</static>
      <pose>{pose}</pose>
      
      <!-- Industrial Warehouse Rack - Inline Definition -->
      <!-- Vertical Posts -->
      <link name='{name}_post_left_1'>
        <pose>-1.4 -17.5 3 0 0 0</pose>
        <collision name='collision'>
          <geometry>
            <box>
              <size>0.12 0.12 6</size>
            </box>
          </geometry>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>0.12 0.12 6</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.5 0.1 1</ambient>
            <diffuse>0.9 0.6 0.2 1</diffuse>
          </material>
        </visual>
      </link>
      
      <link name='{name}_post_left_2'>
        <pose>-1.4 0 3 0 0 0</pose>
        <collision name='collision'>
          <geometry>
            <box>
              <size>0.12 0.12 6</size>
            </box>
          </geometry>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>0.12 0.12 6</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.5 0.1 1</ambient>
            <diffuse>0.9 0.6 0.2 1</diffuse>
          </material>
        </visual>
      </link>
      
      <link name='{name}_post_left_3'>
        <pose>-1.4 17.5 3 0 0 0</pose>
        <collision name='collision'>
          <geometry>
            <box>
              <size>0.12 0.12 6</size>
            </box>
          </geometry>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>0.12 0.12 6</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.5 0.1 1</ambient>
            <diffuse>0.9 0.6 0.2 1</diffuse>
          </material>
        </visual>
      </link>
      
      <link name='{name}_post_right_1'>
        <pose>1.4 -17.5 3 0 0 0</pose>
        <collision name='collision'>
          <geometry>
            <box>
              <size>0.12 0.12 6</size>
            </box>
          </geometry>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>0.12 0.12 6</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.5 0.1 1</ambient>
            <diffuse>0.9 0.6 0.2 1</diffuse>
          </material>
        </visual>
      </link>
      
      <link name='{name}_post_right_2'>
        <pose>1.4 0 3 0 0 0</pose>
        <collision name='collision'>
          <geometry>
            <box>
              <size>0.12 0.12 6</size>
            </box>
          </geometry>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>0.12 0.12 6</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.5 0.1 1</ambient>
            <diffuse>0.9 0.6 0.2 1</diffuse>
          </material>
        </visual>
      </link>
      
      <link name='{name}_post_right_3'>
        <pose>1.4 17.5 3 0 0 0</pose>
        <collision name='collision'>
          <geometry>
            <box>
              <size>0.12 0.12 6</size>
            </box>
          </geometry>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>0.12 0.12 6</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.5 0.1 1</ambient>
            <diffuse>0.9 0.6 0.2 1</diffuse>
          </material>
        </visual>
      </link>
      
      <!-- Shelf Levels -->
      <link name='{name}_shelf_level1'>
        <pose>0 0 1.5 0 0 0</pose>
        <collision name='collision'>
          <geometry>
            <box>
              <size>2.8 35 0.08</size>
            </box>
          </geometry>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>2.8 35 0.08</size>
            </box>
          </geometry>
          <material>
            <ambient>0.6 0.6 0.6 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
      </link>
      
      <link name='{name}_shelf_level2'>
        <pose>0 0 3 0 0 0</pose>
        <collision name='collision'>
          <geometry>
            <box>
              <size>2.8 35 0.08</size>
            </box>
          </geometry>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>2.8 35 0.08</size>
            </box>
          </geometry>
          <material>
            <ambient>0.6 0.6 0.6 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
      </link>
      
      <link name='{name}_shelf_level3'>
        <pose>0 0 4.5 0 0 0</pose>
        <collision name='collision'>
          <geometry>
            <box>
              <size>2.8 35 0.08</size>
            </box>
          </geometry>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>2.8 35 0.08</size>
            </box>
          </geometry>
          <material>
            <ambient>0.6 0.6 0.6 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
      </link>
      
    </model>'''

# Pattern to match include statements for racks
include_pattern = r'    <include>\s*<uri>model://warehouse_rack</uri>\s*<pose>([^<]+)</pose>\s*<name>([^<]+)</name>\s*</include>'

def replace_include(match):
    pose = match.group(1)
    name = match.group(2)
    return industrial_rack_template.format(name=name, pose=pose)

# Replace all include statements with inline industrial rack definitions
new_content = re.sub(include_pattern, replace_include, content, flags=re.MULTILINE)

# Write back to file
with open('/home/saidul/Desktop/fork_ws/src/harmonic_sim/worlds/blueprint_warehouse_detailed.world', 'w') as f:
    f.write(new_content)

print("✅ All rack includes replaced with inline industrial rack definitions!")