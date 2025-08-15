#!/usr/bin/env python3
"""
Visualize the warehouse layout from the blueprint
Shows room divisions, rack positions, and door locations
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches

def create_warehouse_visualization():
    """Create a 2D visualization of the warehouse layout"""
    
    fig, ax = plt.subplots(1, 1, figsize=(14, 10))
    
    # Warehouse boundaries
    warehouse = patches.Rectangle((-82.5, -82.5), 165, 165, 
                                  linewidth=3, edgecolor='black', facecolor='white')
    ax.add_patch(warehouse)
    
    # Room dividers
    # Between 101 and 102
    ax.plot([-24.25, -24.25], [-82.5, -2.5], 'k-', linewidth=2, label='Room Dividers')
    ax.plot([-24.25, -24.25], [2.5, 82.5], 'k-', linewidth=2)
    
    # Between 102 and 104
    ax.plot([25.9, 25.9], [-82.5, -12.5], 'k-', linewidth=2)
    ax.plot([25.9, 25.9], [-7.5, 82.5], 'k-', linewidth=2)
    
    # Between 104 and 105
    ax.plot([34.4, 34.4], [-82.5, 82.5], 'k-', linewidth=2)
    
    # Racks in Room 101
    rack_color = 'gray'
    rack_width = 2.5
    rack_length = 40
    
    # Room 101 racks
    for x in [-60, -55, -50, -45, -40, -35]:
        rack = patches.Rectangle((x-rack_width/2, -rack_length/2), rack_width, rack_length,
                                 linewidth=1, edgecolor='black', facecolor=rack_color)
        ax.add_patch(rack)
    
    # Room 102 racks
    for x in [-17, -12, -7, -2, 3, 8, 13, 18]:
        rack = patches.Rectangle((x-rack_width/2, -rack_length/2), rack_width, rack_length,
                                 linewidth=1, edgecolor='black', facecolor=rack_color)
        ax.add_patch(rack)
    
    # Room 105 racks
    for x in [45, 50, 55, 60, 65, 70]:
        rack = patches.Rectangle((x-rack_width/2, -rack_length/2), rack_width, rack_length,
                                 linewidth=1, edgecolor='black', facecolor=rack_color)
        ax.add_patch(rack)
    
    # Doors
    # Type F doors (industrial roll-up)
    ax.plot([-36.5, -33.5], [-82.5, -82.5], 'g-', linewidth=6, label='Type F Doors')
    ax.plot([33.5, 36.5], [-82.5, -82.5], 'g-', linewidth=6)
    
    # Type D doors (double doors)
    ax.plot([-24.25, -24.25], [-1.8, 1.8], 'r-', linewidth=4, label='Type D Doors')
    ax.plot([25.9, 25.9], [-11.8, -8.2], 'r-', linewidth=4)
    
    # Type C doors (personnel)
    ax.plot([-24.25, -24.25], [-20.5, -19.5], 'b-', linewidth=3, label='Type C Doors')
    ax.plot([25.9, 25.9], [19.5, 20.5], 'b-', linewidth=3)
    
    # Navigation corridors (highlighted)
    nav_color = 'yellow'
    nav_alpha = 0.3
    
    # Main east-west corridor
    nav_corridor = patches.Rectangle((-82.5, -2.5), 165, 5,
                                     linewidth=0, facecolor=nav_color, alpha=nav_alpha)
    ax.add_patch(nav_corridor)
    
    # Room labels
    ax.text(-53, 70, 'Room 101\nFRYS\n2355.6 m²', fontsize=12, ha='center', 
            bbox=dict(boxstyle="round,pad=0.3", facecolor='lightblue'))
    ax.text(0, 70, 'Room 102\nKJØL\n2684.7 m²', fontsize=12, ha='center',
            bbox=dict(boxstyle="round,pad=0.3", facecolor='lightcyan'))
    ax.text(30, 70, 'Room 104\nTØRRVARE\n288.2 m²', fontsize=10, ha='center',
            bbox=dict(boxstyle="round,pad=0.3", facecolor='lightyellow'))
    ax.text(58, 70, 'Room 105\nTØRRVARE\n2300.2 m²', fontsize=12, ha='center',
            bbox=dict(boxstyle="round,pad=0.3", facecolor='lightgreen'))
    
    # Forklift position
    forklift = patches.Circle((0, -40), 2, color='orange', label='Forklift')
    ax.add_patch(forklift)
    
    # Add some example pallets
    pallet_size = 1.2
    for (x, y) in [(-62, 15), (-58, -25), (-15, 20), (10, -45), (28, 10), (48, -25), (63, 40)]:
        pallet = patches.Rectangle((x-pallet_size/2, y-pallet_size/2), pallet_size, pallet_size*0.8,
                                   linewidth=0.5, edgecolor='brown', facecolor='tan')
        ax.add_patch(pallet)
    
    # Settings
    ax.set_xlim(-85, 85)
    ax.set_ylim(-85, 85)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('X (meters)')
    ax.set_ylabel('Y (meters)')
    ax.set_title('Warehouse Layout - Blueprint Implementation\n165m x 165m Total Area', fontsize=16)
    
    # Legend
    ax.legend(loc='upper left', bbox_to_anchor=(1.05, 1), borderaxespad=0.)
    
    # Compass
    ax.annotate('N', xy=(75, 75), xytext=(75, 80), 
                arrowprops=dict(arrowstyle='->', lw=2),
                fontsize=14, ha='center', weight='bold')
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    print("Generating warehouse layout visualization...")
    create_warehouse_visualization()
    print("Visualization complete!")