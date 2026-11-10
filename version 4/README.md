# Version 4: compact tube-arm frame for NIDAR AirMouse

The smallest frame your current parts allow. It is version 3's tube-and-plate build shrunk to fit the AirMouse rules, with version 2's downward fold. Arms are bought aluminium (or carbon) tubes; everything else prints on the Ender 3 V3 SE.

**Status: not yet printed or flown.** All 33 checks in the script pass, but they are geometry checks and estimates only.

## Why it isn't any smaller

Your 2212 920 KV motors on 3S need 10-inch props to lift this drone. The rules require a guard around each prop, and neighbouring guards can't overlap. That puts each motor at least 138 mm from the centre in X and Y, a **391 mm wheelbase**. A smaller frame needs smaller props, and so stronger motors or a 4S battery.

The script works the size out from the prop: change `PROP_IN` and everything resizes, then re-run the checks.

## Compared with version 2

| | Version 2 | Version 4 |
|---|---|---|
| Width (launch limit 610 mm) | 560 mm | 557 mm |
| Space each side of a 1 m corridor | 220 mm (161 at worst yaw) | 222 mm (165 at worst yaw) |
| Folded size | 554 × 272 × 312 mm | 551 × 240 × 274 mm |
| Frame mass | about 730 g | about 650 g (604 g printed + 48 g tubes) |
| Thrust-to-weight | 1.17–1.55 | 1.22–1.62 |
| Hover time | about 8 min | about 8.3 min |
| Arms | printed | bought tubes, cheap to replace after a crash |

**The width barely changes**, because the guards set it in both versions. What version 4 gains is weight, a smaller folded size, and arms that are cheap to replace after a crash.

## Run

```
pip install cadquery ezdxf shapely numpy trimesh pillow    (Python 3.12)
python airmouse_v4_generator.py
```

| Folder | Contents |
|---|---|
| `output/stl` | Print files |
| `output/step` | One STEP per part plus `assembly_deployed.step`, for SOLIDWORKS |
| `output/dxf` | Plate profiles, if you'd rather cut them from 1.5 mm carbon |
| `output/renders` | Pictures of the deployed and folded drone, and one arm |

## Printed parts

All parts fit the Ender bed and need no supports. Keep each STL in the position it loads in. Print in PETG.

| STL | Qty | Size (mm) | About |
|---|---|---|---|
| `arm_root_A_x2` / `arm_root_B_x2` | 2 + 2 | 26 × 42 × 23 | 15 g each |
| `motor_mount_x4` | 4 | 69 × 58 × 24 | 17 g each |
| `hinge_base_A_x2` / `hinge_base_B_x2` | 2 + 2 | 37 × 30 × 24 | 13 g each |
| `top_plate_x1` | 1 | 150 × 150 × 3 | 65 g |
| `bottom_plate_x1` | 1 | 150 × 150 × 3 | 55 g |
| `lidar_deck_x1` | 1 | 92 × 76 × 3 | 21 g |
| `battery_tray_x1` | 1 | 170 × 70 × 47 | 68 g |
| `leg_A_x2` / `leg_B_x2` | 2 + 2 | 81 × 40 × 16 | 9 g each |
| `guard_arc_x16` | 16 | 42 × 192 × 16 | 7 g each |
| `guard_spoke_x16` | 16 | 117 × 46 × 20 | 4 g each |

A parts go at front left and rear right; B parts at front right and rear left.

## Parts to buy

| Item | Qty |
|---|---|
| Aluminium tube 16 mm OD × 1 mm wall (or carbon 16 × 14 mm), cut to **94.3 mm** | 4 |
| M3 × 24 female-female hex standoffs (between the plates) | 2 |
| M3 × 35 female-female hex standoffs (lidar deck) | 4 |
| M4 × 45 + nyloc nut (hinge and lock) | 8 |
| M3 × 35 + nyloc nut (hinge bases through both plates) | 8 |
| M3 × 30 + nyloc nut (tube cross bolts) | 8 |
| M3 × 55, or M3 threaded rod, + nuts (battery tray) | 4 |
| M3 × 12 + nyloc nut (legs, guard spokes) | 40 |
| M3 × 10 + nut (guard arcs) | 32 |
| M3 × 8 (motors: check they stop short of the windings) | 16 |
| M3 × 6 (standoff ends) | 12 |
| M2.5 screws and 6 mm standoffs (Pi 4B, YDLidar X2) | 7 + 4 |

**Drill each tube** with a 3.4 mm hole straight through it, 8 mm from the root end and 80.3 mm from the root end. The cross bolts pass through these, which stops the tube twisting or pulling out.

## Where everything goes

| Part | Location |
|---|---|
| Pixhawk 2.4.8 | Between the plates, on 4 mm foam, with zip-tie slots |
| Raspberry Pi 4B | Hangs under the lidar deck, 7 mm above the top plate |
| YDLidar X2 | On the lidar deck. Scan plane at 110 mm, 37 mm above the guards |
| Battery | In the tray under the bottom plate (up to 165 × 50 × 40 mm); slides in from front or back |
| PX4Flow | Under the tray floor at the front, held by zip ties |
| TFmini-S | Under the tray floor at the rear, held by two M2 screws |
| ESCs | Zip-tied to the tubes |

With the battery hanging low, the centre of gravity is low too, which helps with stability.

## Folding

Remove the lock bolt (the upper of the two hinge bolts) and swing the arm down. Thrust pushes the arm against a solid stop wall, so the lock bolt doesn't carry flight loads. Leave enough wire slack at each hinge for 90° of movement.

## Check results

All 33 checks pass:

- **Printed parts:** all 13 fit the bed and are valid single solids with watertight STLs.
- **Rule fit:** the frame fits the launch square (557 mm wide, 53 mm spare) and a 1 m corridor (165 mm each side at the worst yaw).
- **Clearances:** arms, guards and motors clear each other and the body when deployed, when folded, and at 20°, 45° and 70° through the swing. The arm can't rotate upward past its stop.
- **Props and sensors:** the props clear everything, the X2 scan plane is above the guards, and the downward sensors see past the legs and stay above the ground when landed.
- **Fit:** the Pixhawk, Pi 4B and battery fit. Every hole and slot in the plates leaves at least 3 mm of material.
- **Bolt holes:** the holes for the hinges, plates, tray, deck, legs and guards line up.

Not checked: real strength, vibration, crash loads, and airflow lost to the plates and guards.

## Still to measure before printing everything

The open questions are the same as version 2:

1. **Thrust:** test one motor on a 1045 prop. Below 700 g per motor, this setup won't fly safely.
2. **Motor and prop:** measure the motor bolt pattern and the prop height (`MOTOR_HOLES`, `PROP_ABOVE_PAD`).
3. **Battery:** measure the pack (`BATTERY`).
4. **Guards:** ask the organisers whether ring-only guards meet rule 8.30.
