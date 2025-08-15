#!/usr/bin/env python3
"""
Generate pallet positions for warehouse simulation
This script can be used to generate random pallet positions avoiding rack locations
"""

import random

def generate_pallet_positions():
    """Generate 100 pallet positions distributed across warehouse rooms"""
    pallets = []
    pallet_id = 1
    
    # Room 101-FRYS: x from -70 to -30, y from -70 to 70
    # Racks are at x = -60, -55, -50, -45, -40, -35
    for i in range(25):
        x = random.uniform(-65, -32)
        y = random.uniform(-60, 60)
        # Avoid rack positions (multiples of 5 in x)
        while any(abs(x - rack_x) < 2 for rack_x in [-60, -55, -50, -45, -40, -35]):
            x = random.uniform(-65, -32)
        pallets.append(f'''
    <model name="pallet_{pallet_id}">
      <static>true</static>
      <pose>{x:.1f} {y:.1f} 0 0 0 {random.uniform(0, 6.28):.2f}</pose>
      <include>
        <uri>model://warehouse_pallet</uri>
      </include>
    </model>''')
        pallet_id += 1
    
    # Room 102-KJØL: x from -20 to 20, y from -70 to 70
    # Racks are at x = -17, -12, -7, -2, 3, 8, 13, 18
    for i in range(30):
        x = random.uniform(-20, 20)
        y = random.uniform(-60, 60)
        # Avoid rack positions
        rack_positions = [-17, -12, -7, -2, 3, 8, 13, 18]
        while any(abs(x - rack_x) < 2 for rack_x in rack_positions):
            x = random.uniform(-20, 20)
        pallets.append(f'''
    <model name="pallet_{pallet_id}">
      <static>true</static>
      <pose>{x:.1f} {y:.1f} 0 0 0 {random.uniform(0, 6.28):.2f}</pose>
      <include>
        <uri>model://warehouse_pallet</uri>
      </include>
    </model>''')
        pallet_id += 1
    
    # Room 104-TØRRVARE: x from 26 to 34, y from -70 to 70
    for i in range(10):
        x = random.uniform(27, 33)
        y = random.uniform(-60, 60)
        pallets.append(f'''
    <model name="pallet_{pallet_id}">
      <static>true</static>
      <pose>{x:.1f} {y:.1f} 0 0 0 {random.uniform(0, 6.28):.2f}</pose>
      <include>
        <uri>model://warehouse_pallet</uri>
      </include>
    </model>''')
        pallet_id += 1
    
    # Room 105-TØRRVARE: x from 40 to 80, y from -70 to 70
    # Racks are at x = 45, 50, 55, 60, 65, 70
    for i in range(35):
        x = random.uniform(42, 78)
        y = random.uniform(-60, 60)
        # Avoid rack positions
        rack_positions = [45, 50, 55, 60, 65, 70]
        while any(abs(x - rack_x) < 2 for rack_x in rack_positions):
            x = random.uniform(42, 78)
        pallets.append(f'''
    <model name="pallet_{pallet_id}">
      <static>true</static>
      <pose>{x:.1f} {y:.1f} 0 0 0 {random.uniform(0, 6.28):.2f}</pose>
      <include>
        <uri>model://warehouse_pallet</uri>
      </include>
    </model>''')
        pallet_id += 1
    
    return '\n'.join(pallets)

if __name__ == "__main__":
    print("<!-- Generated pallets -->")
    print(generate_pallet_positions())
    print("\n<!-- Total pallets: 100 -->")