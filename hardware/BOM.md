# Bill of Materials (BOM)

| Component             | Quantity | Description                              | Cost (per unit) |
|-----------------------|----------|------------------------------------------|-----------------|
| NEMA17 stepper motor  | 3        | 1.8° step angle, 12V, 1.5A               | € 8.88          |
| Arduino Uno           | 1        |                                          | €22.99          |
| CNC shield            | 1        | Arduino Uno shield for CNCs              | €16.79          |
| Stepper motor driver  | 3        | Stepper drivers based on A4988           | € 1.79          |
| Timing belt           | 3        | GT2, 6mm width, 280mm long               | € 1.92          |
| Lead screw            | 1        | 8mm diameter, 300mm length, with POM nut | € 5.50          |
| Aluminum rods         | 3        | 8mm diameter, 300mm length               | € 9.99          |
| Pulley                | 3        | GT2 for 6mm belt, 16 teeth, 5mm bore     | € 1.36          |
| Coupler               | 1        | 5mm to 8mm coupler                       | € 1.60          |
| LM8LUU linear bearing | 3        | Linear bearing, 8mm x 15mm x 45mm        | € 2.40          |
| 6002-2RS              | 3        | Ball bearing, 15mm x 32mm x 9mm          | € 1.42          |
| M3 hex screw kit      | 1        | See notes                                | €11.99          |
| M2 hex screw kit      | 1        | See notes                                | € 7.99          |
| Heat set inserts      | 1        | See notes                                | € 0.06          |
| Locknuts              | 1        | See notes                                | € 0.06          |
| Micro switches        | 3        |                                          | € 0.22          |

Total: 150.8 (omitting heat set inserts and locknuts)

## Notes

- Keep in mind that the cost per unit, and consequently the total cost, may vary depending on where you source the parts. For this reason I have not included sourcing links. The listed unit prices reflect now much I paid for each item.
- I did not include zip ties, spiral wraps for the cables and the labels (for which you will also need a labeling machine).
- The aluminum rods I bought are hollow, but this doesn't matter. It's just what was available at the time.
- I added a 12V fan to cool down the stepper drivers but this is totally optional and thus omitted from the BOM. The fan was too loud so I also added a step down converter to decrease the voltage, also optional. If you don't have a step down laying around you can also:
    1. use the fan connecting it directly to the power jack if it's 12V;
    2. use a variable resistor to create a voltage divider and lower its input voltage; this is kind of sketchy and the resistor can get very hot, not recommended;
    3. use a 5V fan and connect it to the 5V and GND pins of the CNC shield;
    
    I provided 3 versions of the control box lid to accomodate each case.
- Number of locknuts used (only where vibrations could break the arm): 8
- Number of heat set inserts used: 24
- Complete list of M2 screws and nuts used: 6x M2x16 screw + 6x M2 nuts to keep the micro switches in place 
- Complete list of M3 screws and nuts used:
        2x M3x35
        3x M3x10 (control box lid)
        2x M3x10 (mount arduino to control box)
        2x M3x10 (join control box to base)
        5x M3x10 (forearm)
        10x M3x8 (two z_base and lead screw nut assembly)
        6x M3x16 (limit switches) 
        8x M3 locknuts (limit switches and bottom of the forearm)
        optional - 4x M3x18 (mount fan)
        optional - 2x M3x16 (mount step down voltage regulator)
