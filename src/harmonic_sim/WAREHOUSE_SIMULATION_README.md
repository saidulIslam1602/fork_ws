# Blueprint Warehouse 3D Simulation

This simulation recreates the warehouse layout from the architectural drawing with accurate dimensions and features.

## Features Implemented

### 1. Accurate Layout and Areas
- **Room 101-FRYS**: 2355.6 m² (Freezer storage)
- **Room 102-KJØL**: 2684.7 m² (Cold storage)
- **Room 104-TØRRVARE**: 288.2 m² (Dry goods - small)
- **Room 105-TØRRVARE**: 2300.2 m² (Dry goods - large)
- Total warehouse dimensions: 165m x 165m

### 2. Navigation Areas
- Clear corridors between rack rows (5m wide)
- Main navigation corridor running east-west
- Navigation gaps in room dividers for inter-room movement
- Yellow navigation line markers for visual guidance

### 3. Functional Doors
- **Type C Doors** (Personnel): 0.9m wide, hinged, blue color
  - Located at room dividers for personnel access
- **Type D Doors** (Double): 1.8m wide (2x 0.9m), red color
  - Located between rooms for equipment passage
- **Type F Doors** (Industrial Roll-up): 3m wide, dark grey
  - Located at south wall for truck loading/unloading
  - F11 and F12 doors positioned as per drawing

### 4. Room Dividers
- Dividers between rooms with navigation gaps
- Does not block forklift movement
- Positioned according to the architectural drawing

### 5. Industrial Racks
- North-South orientation (along Y-axis)
- Dimensions: 2.5m wide x 40m long x 8m tall
- 3 shelf levels at 2m, 4m, and 6m heights
- Distribution:
  - Room 101: 6 rack rows
  - Room 102: 8 rack rows
  - Room 104: No racks (small room)
  - Room 105: 6 rack rows

### 6. Forklift
- Positioned in central navigation area
- Uses existing forklift model from the project
- Ready for navigation and manipulation tasks

### 7. Pallets
- 100 pallets distributed throughout the warehouse
- Standard EUR pallet size: 1.2m x 0.8m x 0.144m
- Each pallet includes cargo boxes with strapping
- Randomly positioned avoiding rack locations
- Distribution:
  - Room 101: 25 pallets
  - Room 102: 30 pallets
  - Room 104: 10 pallets
  - Room 105: 35 pallets

## File Structure

```
harmonic_sim/
├── worlds/
│   └── blueprint_warehouse_detailed.world    # Main world file
├── models/
│   ├── warehouse_rack/                       # Industrial rack model
│   │   ├── model.config
│   │   └── model.sdf
│   └── warehouse_pallet/                     # Pallet with cargo model
│       ├── model.config
│       └── model.sdf
├── scripts/
│   └── generate_pallets.py                   # Script to generate pallet positions
├── launch/
│   └── blueprint_warehouse_detailed.launch.py # Launch file
└── test_blueprint_warehouse.sh               # Test script
```

## Running the Simulation

1. Build the workspace:
```bash
cd /home/saidul/Desktop/fork_ws
colcon build --packages-select harmonic_sim
source install/setup.bash
```

2. Launch the simulation:
```bash
ros2 launch harmonic_sim blueprint_warehouse_detailed.launch.py
```

Or use the test script:
```bash
./src/harmonic_sim/test_blueprint_warehouse.sh
```

## Customization

### Adding More Pallets
Use the `generate_pallets.py` script to generate new random pallet positions:
```bash
python3 src/harmonic_sim/scripts/generate_pallets.py
```

### Modifying Rack Layout
Edit the rack positions in the world file. Racks are positioned at 5m intervals.

### Adjusting Door Behavior
Doors use joints for movement:
- Type C: Revolute joint (0 to 90 degrees)
- Type D: Fixed position (can be made dynamic)
- Type F: Prismatic joint (0 to 3.5m vertical)

## Navigation Considerations

- Navigation area width: 5m between racks
- Door clearance: Minimum 3m for Type F doors
- Turn radius consideration for forklift movement
- Clear sight lines maintained throughout warehouse

## Performance Notes

- 100 pallets may impact simulation performance on lower-end systems
- Consider reducing pallet count if experiencing lag
- Static racks help maintain good performance
- World file uses embedded models to reduce loading time