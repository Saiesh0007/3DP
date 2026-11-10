# Version 6: X frame in plywood and aluminium for NIDAR AirMouse

This is a true X frame, like a ZD550 scaled down to fit the AirMouse rules, and the arms fold down. **Nothing is 3D printed and nothing is carbon:**

- The plates are birch plywood, cut on any laser cutter.
- The arms are aluminium square tube from a hardware shop.
- The prop guards are polycarbonate strip bands on plywood spokes.

**Status: not yet cut, built or flown.** All 42 checks in the script pass, but they cover geometry and hand calculations only.

## How it is laid out

- **Arms on the diagonals.** Four 12 × 12 mm aluminium tubes run out at 45° from a small centre hub. The motors sit on the same square as versions 4 and 5 (±139 mm), so ArduPilot or PX4 flies it as a normal Quad X.
- **Arms fold down and to the side.** The hinge bolt runs front to back, as in version 4.
  - Each tube passes at 45° through rectangular slots in two upright plywood "knuckle" plates. The knuckles swing between two plywood "cheeks" fixed to the hub.
  - A laser cuts the angled slots, so the skewed hinge needs no printed or machined block.
- **Thrust stop.** In flight, thrust presses the knuckles' top edges against the top plate, so the lock bolt carries no flight load. Remove the lock bolt to fold.
- **Guards.** A 1 mm polycarbonate band is held by four plywood spokes, which slot into the motor plate. The spoke lugs pass through slots in the band and are held by a zip tie or a 2 mm pin.

## Against the rules

| Rule | How version 6 meets it |
|---|---|
| No ready-made airframes | Your own design, made from sheet, tube and strip. Only standoffs, bolts and electronics are bought. |
| 1 m corridors | 222 mm to each wall flying straight, 164 mm at the worst 45° yaw |
| 2 ft launch square | 556 mm wide, 54 mm spare |
| Live camera feed | Front camera plate for the Pi Camera 3. The check shows its 66 × 41° view clears the guards, arms and legs. |
| Guarded props | A band around every prop. **Ask the organisers whether ring guards count as fully protecting the props.** |
| 10 kg limit | About 2.0 kg |

## Compared with versions 4 and 5

| | Version 4 | Version 5 | Version 6 |
|---|---|---|---|
| Layout | Tube arms, H body | Carbon H spine | True X |
| Materials | PETG prints, aluminium tube | Carbon plate, PETG guards | Plywood, aluminium tube, polycarbonate |
| Needs | 3D printer | CNC carbon service | Laser cutter and a hacksaw |
| Frame mass | about 650 g | about 840 g | about 640 g |
| All-up weight | about 2.1 kg | about 2.2 kg | about 2.0 kg |
| Thrust-to-weight | 1.22–1.62 | 1.16–1.53 | 1.29–1.71 |
| Hover time | about 8.3 min | about 7.9 min | about 8.7 min |
| Folded size | 551 × 240 × 274 mm | 551 × 158 × 360 mm | 552 × 192 × 355 mm |

**Version 6 is the lightest of the three and the cheapest to make and repair.** Plywood is weaker than carbon, so the lowest safety factor in the hand calculations is 4 (version 5: 12). That's fine for flight loads, but a hard crash will break plywood parts sooner. Keep spares; they cost very little to cut.

## Run

```
pip install cadquery ezdxf shapely numpy trimesh pillow    (Python 3.12)
python airmouse_v6_x_generator.py
```

The run takes about 15 minutes, mostly for the renders. Set `V6_NO_RENDER=1` to run only the checks.

| Folder | Contents |
|---|---|
| `output/dxf` | One DXF per plywood part, `CUT_SHEET_4mm.dxf` and `CUT_SHEET_3mm.dxf` with every part nested, and `guard_band_polycarbonate_x4.dxf` |
| `output/step` | One STEP per part, the tube and band, plus `assembly_deployed.step` and `assembly_folded.step`, for SOLIDWORKS |
| `output/renders` | Deployed, folded, fold sequence, top view in the launch square, front view in a 1 m corridor, arm and hinge close-up, exploded view, and both cut sheets |

DXF layers: `CUT_OUTER` is the outside profile, `CUT_INNER` is holes and slots, and `NOTES` holds labels only. Units are millimetres.

## Plywood parts (laser cut)

| Part | Sheet | Qty | Size (mm) | Each |
|---|---|---|---|---|
| `top_plate` | 4 mm | 1 | 312 × 132 | 59 g |
| `bottom_plate` | 4 mm | 1 | 312 × 92 | 37 g |
| `cheek` | 4 mm | 8 | 38 × 48 | 4 g |
| `knuckle_inner` | 4 mm | 4 | 40 × 30 | 2 g |
| `knuckle_outer` | 4 mm | 4 | 68 × 30 | 4 g |
| `leg` | 4 mm | 2 | 104 × 92 | 8 g |
| `motor_plate` | 3 mm | 4 | 86 × 80 | 6 g |
| `tip_plate` | 3 mm | 4 | 71 × 50 | 5 g |
| `guard_spoke` | 3 mm | 16 | 128 × 49 | 3 g |
| `lidar_deck` | 3 mm | 1 | 92 × 74 | 12 g |
| `sensor_plate` | 3 mm | 1 | 216 × 72 | 22 g |
| `camera_plate` | 3 mm | 1 | 32 × 40 | 2 g |

The nested parts need a 600 × 307 mm sheet of 4 mm ply and a 600 × 373 mm sheet of 3 mm ply.

**When ordering:**

- **Material:** use birch plywood (often sold as "aircraft" or "laser" ply). Avoid MDF and poplar, which are weaker.
- **Sharp corners:** a laser cuts square inside corners, so the tab slots need no dogbone reliefs.
- **Material around holes:** every hole and slot leaves at least 3 mm of wood to the next hole and to the edge.
- **Slot clearance:** slots are 0.15 mm larger than their tabs. If tabs are loose or tight after a test cut, adjust `SLOT_PLAY`.
- **Measure the sheet first.** "4 mm" ply is often 3.7–3.9 mm. Set `T4` and `T3` to the real thickness, then re-run before cutting.
- **Seal the wood:** a coat of thin CA glue or varnish on the hinge and slot edges stops them wearing and keeps damp out.

## Aluminium tube (4 arms)

12 × 12 mm square tube with 1 mm walls (6063). For each arm:

1. **Cut to length.** The root end is cut at 45°, so one side is 117.9 mm long and the other 105.9 mm. A miter box makes this easy. The outer end is cut square.
2. **Drill four 3.4 mm holes** straight down through both walls, on the centre line. Measure each from the square outer end:
   - 10.0 mm and 26.0 mm: clamp bolts (motor plate, tube and tip plate).
   - 76.9 mm and 101.5 mm: retaining pins. Their heads bear on the knuckle plates, so the tube can't slide.

## Polycarbonate guard bands (4)

- **Size:** 1 mm polycarbonate sheet, cut into strips 878 × 16 mm.
- **Holes and slots:** mark them from `guard_band_polycarbonate_x4.dxf`.
  - A 3.3 × 10.3 mm slot for each spoke lug.
  - Two M3 holes at each end for the overlap joint.
- **Cutting:** use a knife and straight edge, then drill and file. **Don't laser-cut polycarbonate**: it gives off harmful fumes and leaves a poor edge.
- **Joining:** bend the strip into a ring, overlap the ends by 20 mm and bolt them with two M3 × 6 screws and nuts.

## Parts to buy

| Item | Qty | Where |
|---|---|---|
| M3 × 32 hex standoffs | 12 | Between the hub plates |
| M3 × 35 hex standoffs | 4 | Lidar deck |
| M3 × 45 hex standoffs | 4 | Battery cage |
| M3 × 12 hex standoffs | 8 | Motor plate to tip plate, beside the tube end |
| M4 × 45 bolt + nyloc nut, with a 7 × 4.2 × 22 mm aluminium or brass tube | 4 | Hinge bolts |
| M3 × 45 bolt + nyloc nut, with a 6 × 3.2 × 22 mm tube | 4 | Lock bolts (remove to fold) |
| M3 × 45 bolt + nut, with a 6 × 3.2 × 33 mm tube | 8 | Cheek spacers |
| M3 × 45 bolt + nut, with a 6 × 3.2 × 33 mm tube | 4 | Tab locks above the top plate |
| 0.5 mm nylon washers, M4 and M3 | 16 | Between knuckles and cheeks |
| M3 × 20 + nyloc nut | 16 | Clamp bolts (8) and tube retaining pins (8) |
| M3 × 10 + nut | 16 | Spokes into the motor plate (the nut sits in the spoke's T-slot) |
| M3 × 6 + nut | 8 | Band overlap joints |
| M3 × 6 button-head screws | about 60 | Standoff ends |
| M3 × 8 | 16 | Motors (check they stop short of the windings) |
| M3 × 12 + nyloc nut | 2 | Leg cross pins |
| Small zip ties or 2 mm pins | 16 | Spoke lugs outside the band |
| M2 screws, M2.5 screws and standoffs | | TFmini-S, Pi 4B, YDLidar X2, Pi Camera |
| 20 mm velcro straps | 2 | Battery |

## Assembly

1. **Hub.**
   - Push the cheeks' bottom tabs into the bottom plate.
   - Fit the M3 × 32 standoffs, then the top plate over the cheeks' top tabs.
   - Fit each pair of cheeks' cheek spacers. Fit the tab-lock spacer above the top plate, through the holes in the tabs; it clamps the top plate down.
2. **Arms.**
   - Slide the tube through both knuckles until its 45° end is flush with the inner knuckle's inner face.
   - Fit the two retaining pins through the tube. Their heads sit against the knuckles.
   - Put the hinge spacer and lock spacer between the knuckles.
3. **Motor end.**
   - Bolt the motor to the motor plate from below.
   - Put the motor plate on top of the tube and the tip plate under it.
   - Fit the two M3 × 12 standoffs beside the tube end, then the two clamp bolts through the tube.
4. **Arm into the hub.** Put a washer on each side of the knuckles. Push the hinge bolt and the lock bolt through cheek, knuckles and cheek.
5. **Guards.**
   - Drop a nut into each spoke's T-slot.
   - Stand the spokes in the motor plate slots and screw them in from below.
   - Feed the band over the four lugs, close the overlap joint, and fit a zip tie or pin through each lug outside the band.
6. **Battery cage and electronics:** the same as version 5.
   - Push the legs up through the sensor plate into the bottom plate, then fit the cage standoffs and cross pins.
   - Pixhawk on foam between the plates; Pi 4B under the lidar deck, X2 on top.
   - PX4Flow (front) and TFmini-S (rear) under the sensor plate.
   - Battery strapped between the legs; camera on the front plate.
   - ESCs zip-tied under the tubes.

**Folding:** remove the lock bolt and swing the arm down. It moves down and out to the side. Leave enough wire slack at each hinge for 90° of movement.

## Check results

All 42 checks pass:

- **Every plywood part:**
  - is a valid single solid
  - has every hole inside it
  - keeps at least 3 mm of wood around every hole and slot
- **Rule fit:** the launch square, the corridor (straight and at the worst yaw) and the 10 kg limit.
- **Joints:** plates, cheeks, legs, standoffs, the tube through both knuckles, spokes and band all seat without clashing.
- **Clearances:**
  - Arms, guards and motors clear the hub and each other when deployed, when folded, and at 20°, 45° and 70° through the swing.
  - The thrust stop works, and with the lock bolt in, the arm can't fold.
- **Props and sensors:**
  - The props clear everything.
  - The X2 scan plane, at 120 mm, is above the guards.
  - The camera view is clear.
  - The downward sensors see past the legs and stay 19 mm above the ground.
- **Bolt holes:** every bolt line passes through every part it should: hinges, locks, cheek spacers, tab locks, all standoffs, clamp bolts, tube pins, motor screws and leg pins.
- **Airflow:** everything under the prop blocks 14% of each prop disc: the arm 7.8%, the hub, battery and legs 3.5%, and the guard spokes 3.0%.
- **Strength:** hand calculations at full thrust × 2.
  - Tube bending: safety factor 12.
  - Motor deflection: 0.05 mm.
  - Hinge bolt bearing in the plywood: safety factor 7.
  - Top plate between the cheeks (where the stop pushes): safety factor 4.

**Not checked:** crash loads, vibration modes, and the real strength of your plywood.

## Still to measure before cutting

1. **Thrust:** test one motor with a 1045 prop. Below 700 g per motor, this setup won't fly safely.
2. **Motor and prop:** measure the motor bolt pattern (`MOTOR_HOLES`) and the prop height (`PROP_ABOVE_PAD`).
3. **Battery:** measure the pack (`BATTERY`). The cage takes up to 165 × 50 × 40 mm.
4. **Plywood:** measure the real thickness of both sheets and set `T4` and `T3`.
5. **Guards:** ask the organisers whether ring guards meet the guard rule.
