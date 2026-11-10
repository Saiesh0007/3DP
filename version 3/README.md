# Version 3: ZD550-style frame files

`zd550_generator.py` writes a 2D DXF (for cutting) and a 3D STEP file for each part of a 550 mm folding frame with round boom tubes. It uses only open-source libraries, and all dimensions are variables at the top of the script.

## Setup

CadQuery does not support Python 3.14 yet, so use Python 3.12:

```
pip install cadquery ezdxf
python zd550_generator.py
```

| Library | Used for |
|---|---|
| `cadquery` (installs the OpenCascade CAD kernel) | STEP solids |
| `ezdxf` | DXF profiles, as true lines, arcs and circles in millimetres |

## Output files

| File | Part | Qty |
|---|---|---|
| `zd550_top_plate.dxf` / `.step` | Top plate, 150 × 150 × 1.5 mm | 1 |
| `zd550_bottom_plate.dxf` / `.step` | Bottom plate (same outline) | 1 |
| `zd550_arm.step` | Boom tube, Ø20 × 1 mm wall, 180 mm long | 4 |
| `zd550_arm_cut_sheet.dxf` | Tube section and cut length, for the workshop | |
| `zd550_motor_mount.dxf` / `.step` | Motor mount, Ø50 × 1.5 mm | 4 |

DXF layers: `CUT_OUTER` (red) is the outside profile, `CUT_INNER` (blue) is holes and slots, and `NOTES` holds labels only.

## Boom length

The 550 mm wheelbase runs diagonally between motor axes, so each motor sits 275 mm from the plate centre, along the arm line.

| Point on the arm line | Distance from the plate centre |
|---|---|
| Plate corner, after the 20 mm chamfer | 91.9 mm |
| Hinge pivot (`HINGE_PIVOT_RADIUS`) | 95 mm |
| Inner tube end (pivot + `HINGE_TUBE_START`) | 107 mm |
| Motor axis | 275 mm |
| Outer tube end (motor + `TUBE_PAST_MOTOR`) | 287 mm |

**Tube length = 287 − 107 = 180 mm.** Measure your real hinge and motor clamp, then update the three variables named above.

## Motor mount

Four radial slots reach any bolt circle from 16 to 32 mm. That covers your 2212 motors (16 and 19 mm) and common 3508 and 4108 patterns (25 to 30 mm). There are also four fixed holes on a 25 mm circle, set 45° from the slots. Measure your motor's pattern before cutting.

## Checks

All of these passed:

- Plates: DXF outline is closed and 150 × 150 mm, units are millimetres, and the four corner reliefs are there. The 30 mm grid has four 2.0 mm holes and the 47 mm grid has four 3.0 mm holes.
- STEP volumes match hand calculations: plates 32 403.9 mm³, tube 10 744.2 mm³ (180.00 mm long, Ø20.00), mount 2 571.5 mm³.
- Rebuilding the wheelbase from the tube length gives 550.0 mm.
- Mount: the thinnest web is 2.4 mm between a slot and a hole, and 3.2 mm to the outer edge.

The files haven't been opened in a cutting service's software or in SOLIDWORKS.

## Problems for NIDAR AirMouse

1. **It does not fit the 2 ft launch square (rule 8.23).** Each motor is 194.5 mm out from the centre in each direction.
   - With guarded 10-inch props, the drone is 668 mm wide, against a 610 mm limit.
   - Without guards it is still 643 mm wide.
   - To fit at 550 mm, guards can be no more than about 110 mm in radius, which means props of about 7 inches. That is too small for 2212 920 KV motors on 3S.
2. **Corridor clearance is tight.** At the worst 45° yaw, the drone with guards is 829 mm wide, leaving 85 mm each side of a 1 m corridor. Version 2 leaves 161 mm.
3. **Rule 8.2 bans ready-made complete airframes.** The ZD550 is a commercial frame, so a copy of it may be questioned. Check with the organisers.
4. **1.5 mm plates are meant for carbon fibre or G10 sheet, not 3D printing.** Printed at 1.5 mm, PETG or PLA plates would be far too flexible. For printing, use version 2, or raise `PLATE_THICKNESS` to at least 4 mm.
5. **The 47 mm flight controller grid doesn't fit your Pixhawk 2.4.8.** It has no mounting holes, so it needs foam pads, as in version 2.

To make this frame fit the launch square, lower `WHEELBASE`. A wheelbase of 400 mm or less fits with guarded 10-inch props.
