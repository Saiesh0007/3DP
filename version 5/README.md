# Version 5: carbon fibre plate frame for NIDAR AirMouse

Every structural part of this frame is a flat carbon fibre plate, cut by CNC from 2 mm and 3 mm sheet. The plates are held together by tabs, slots and aluminium standoffs, so no glue is needed. The only printed parts are the prop guards, which are the same as version 4's.

**Status: not yet cut, built or flown.** All the checks in the script pass, but they cover geometry and hand calculations only.

## How it is laid out

- **The body is a long, narrow "H" spine.** Two 3 mm plates run front to back (front is +X) with the Pixhawk between them.
- **The arms fold down.** Each end of the spine has a carbon clevis made of two upright cheek plates. Each clevis carries two arms that run out sideways and fold down about a bolt, like version 2 and version 4.
- **Each arm is a box.** It is two upright 3 mm side plates held 25 mm apart by standoffs. The motor plate and tip plate are clamped onto the side plates' tabs by four standoffs. An upright plate is very stiff against prop thrust, and a damaged side plate is cheap to recut.
- **The motors are on the same square as version 4 (±138.2 mm).** PX4 flies it as a normal Quad X. The launch square fit and corridor clearance are also the same as version 4.
- **The body stays out from under the props.** Body, battery and legs block 2.4% of each prop disc.

## Against the rules

| Rule | How version 5 meets it |
|---|---|
| 8.2: no ready-made airframes | Your own design, cut from sheet. Only parts are bought: standoffs, bolts, motors and electronics. |
| 8.22: 1 m corridors | 222 mm to each wall flying straight, 165 mm at the worst 45° yaw |
| 8.23: 2 ft launch square | 557 mm wide, 53 mm spare |
| 8.27: live camera feed | A front camera plate takes a Pi Camera. The check shows its 62 × 48° view is clear of guards, arms and legs. |
| 8.29 / 8.30: safe indoors, guarded props | A full ring guard on every prop. **Ask the organisers whether ring guards meet 8.30.** |
| 10 kg limit | About 2.2 kg |

## Compared with version 4

| | Version 4 | Version 5 |
|---|---|---|
| Material | Printed PETG with aluminium tube arms | Carbon fibre plates; guards printed |
| Width | 557 mm | 557 mm |
| Folded size | 551 × 240 × 274 mm | 551 × 158 × 360 mm (flatter, taller) |
| Frame mass | about 650 g | about 840 g (427 g carbon, 179 g guards, 234 g standoffs and bolts) |
| All-up weight | about 2.1 kg | about 2.2 kg |
| Thrust-to-weight | 1.22–1.62 | 1.16–1.53 |
| Hover time | about 8.3 min | about 7.9 min |
| Strength | PETG: creeps when warm and can crack | Carbon: the lowest calculated safety factor is 12 |

**Version 5 is the stronger and more repeatable frame, but it is heavier.** The extra 190 g costs about 0.06 in thrust-to-weight. With your motors that matters, so do the thrust test below before ordering the carbon.

## Run

```
pip install cadquery ezdxf shapely numpy trimesh pillow    (Python 3.12)
python airmouse_v5_cf_generator.py
```

The run takes a few minutes, mostly for the renders. Set `V5_NO_RENDER=1` to run only the checks.

| Folder | Contents |
|---|---|
| `output/dxf` | One DXF per carbon part, plus `CUT_SHEET_3mm.dxf` and `CUT_SHEET_2mm.dxf` with every part nested, ready to send for CNC |
| `output/step` | One STEP per part, plus `assembly_deployed.step` and `assembly_folded.step`, for SOLIDWORKS |
| `output/stl` | Guard arcs and spokes to print |
| `output/renders` | Deployed, folded, fold sequence, top view in the launch square, front view in a 1 m corridor, arm and hinge close-up, exploded view, and both cut sheets |

DXF layers: `CUT_OUTER` is the outside profile, `CUT_INNER` is holes and slots, and `NOTES` holds labels only. Units are millimetres.

## Carbon parts

| Part | Sheet | Qty | Size (mm) | Each |
|---|---|---|---|---|
| `top_plate` | 3 mm | 1 | 340 × 92 | 91 g |
| `bottom_plate` | 3 mm | 1 | 340 × 92 | 69 g |
| `cheek` | 3 mm | 4 | 66 × 39 | 8 g |
| `arm_side` | 3 mm | 8 | 154 × 24 | 10 g |
| `leg` | 3 mm | 2 | 104 × 90 | 13 g |
| `motor_plate` | 2 mm | 4 | 66 × 66 | 9 g |
| `tip_plate` | 2 mm | 4 | 54 × 54 | 8 g |
| `lidar_deck` | 2 mm | 1 | 92 × 74 | 19 g |
| `sensor_plate` | 2 mm | 1 | 202 × 72 | 35 g |
| `camera_plate` | 2 mm | 1 | 32 × 31 | 3 g |

The nested parts need a 400 × 451 mm sheet of 3 mm and a 400 × 206 mm sheet of 2 mm.

**When ordering:**

- Ask for 3K twill.
- Cut with a 1.5 mm or smaller end mill. The tab slots have dogbone corners, so square tabs seat fully.
- Every hole and slot leaves at least 3 mm of carbon to the next hole and to the edge.
- Slots are 0.2 mm larger than their tabs. If your cutter's kerf differs, adjust `SLOT_PLAY`.

## Printed parts (PETG, as version 4)

| STL | Qty |
|---|---|
| `guard_arc_x16` | 16 |
| `guard_spoke_x16` | 16 |

## Parts to buy

| Item | Qty | Where |
|---|---|---|
| M3 × 25 hex standoffs | 12 + 8 | Between the body plates; inside the arms |
| M3 × 15 hex standoffs | 16 | Motor plate to tip plate |
| M3 × 35 hex standoffs | 4 | Lidar deck |
| M3 × 45 hex standoffs | 4 | Battery cage |
| M3 × 32 spacers (6 mm aluminium tube, or M3 × 30 standoff + washers) | 8 + 4 | Between the cheeks; tab locks above the top plate |
| M4 × 45 bolt + nyloc nut, with a 7 × 4.2 × 25 mm aluminium tube | 4 | Hinge bolts |
| M3 × 45 bolt + nyloc nut, with a 6 × 3.2 × 25 mm tube | 4 | Lock bolts (remove to fold) |
| 0.5 mm PTFE or nylon washers, M4 and M3 | 16 | Between arm and cheeks |
| M3 × 6 button-head screws (M3 × 8 through 3 mm plates) | about 120 | Standoff and spacer ends |
| M3 × 12 + nyloc nut | 32 + 2 | Guard spokes; leg cross pins |
| M3 × 8 | 16 | Motors (check they stop short of the windings) |
| M2 screws, M2.5 screws and standoffs | | TFmini-S, Pi 4B, YDLidar X2, Pi Camera |
| 20 mm velcro straps | 2 | Battery |

## Assembly

1. **Arms.** Fit two side plates to the M3 × 25 arm standoffs. Seat the motor plate on the top tabs and bolt the motor on from below. Fit the tip plate onto the bottom tabs and clamp both plates with the four M3 × 15 standoffs. To swap a motor, remove the tip plate.
2. **Clevis.** Put the cheeks' top tabs through the top plate. Fit the two "tab lock" spacers above the plate, through the holes in the tabs; they clamp the plate down. Fit the two spacers between the cheeks. The cheeks' bottom tabs locate in the bottom plate strip.
3. **Arms into the clevis.** Fit each arm with a washer on each side. Fit the hinge bolt through its 25 mm tube, then the lock bolt through its tube.
4. **Battery cage.** Push each leg up from below, through the sensor plate, until its top tab is in the bottom plate. Fit the M3 × 45 cage standoffs. Push a cross pin (M3 × 12 and nut) through each leg just above the sensor plate.
5. **Electronics.**
   - Pixhawk on 4 mm foam between the plates.
   - Pi 4B under the lidar deck, X2 on top.
   - PX4Flow (front) and TFmini-S (rear) under the sensor plate.
   - Battery between the legs, strapped down. It slides in from the side.
   - Camera on the front camera plate.
   - ESCs zip-tied under the arms.

**Folding:** remove the lock bolt and swing the arm down. In flight, thrust presses the arm's top edge against the top plate, so the lock bolt carries no flight load. Leave enough wire slack at each hinge for 90° of movement.

## Check results

All checks pass:

- **Every carbon part:**
  - is a valid single solid
  - has every hole inside it
  - keeps at least 3 mm of material around every hole and slot
- **Rule fit:** the launch square, the corridor (straight and at the worst yaw) and the 10 kg limit.
- **Joints:** plates, cheeks, legs, standoffs and arm parts all seat in their slots without clashing.
- **Clearances:**
  - Arms, guards and motors clear the body and each other when deployed, when folded, and at 20°, 45° and 70° through the swing.
  - The thrust stop works, and with the lock bolt in, the arm cannot fold.
- **Props and sensors:**
  - The props clear everything.
  - The X2 scan plane, at 110 mm, is above the guards.
  - The camera view is clear.
  - The downward sensors see past the legs and stay 19 mm above the ground.
- **Bolt holes:** every bolt line passes through every part it should: hinges, locks, cheek spacers, all standoffs, motor screws, guard spokes and leg pins.
- **Airflow:** the body, battery and legs block 2.4% of each prop disc, and everything together blocks under 20%. The arm blocks 8.3% and the guard spokes 4.2%.
- **Strength:** hand calculations at full thrust × 2.
  - Arm bending: safety factor about 70.
  - Hinge bolt bearing: 14.
  - Top plate at the clevis: 12.
  - Motor deflection: 0.04 mm.

**Not checked:** crash loads, vibration modes, and the real strength of your sheet.

## Still to measure before cutting

1. **Thrust:** test one motor with a 1045 prop. Below 700 g per motor, this setup won't fly safely. Version 5 has less margin than version 4.
2. **Motor and prop:** measure the motor bolt pattern (`MOTOR_HOLES`) and the prop height (`PROP_ABOVE_PAD`).
3. **Battery:** measure the pack (`BATTERY`; the cage takes up to 165 × 50 × 45 mm).
4. **Guards:** ask the organisers whether ring guards meet rule 8.30.
5. **Sheet:** measure the real thickness of the sheet you buy. If it isn't exactly 2.0 or 3.0 mm, set `T2` and `T3` before cutting, so the tabs fit their slots.
