# Editing the frame in SOLIDWORKS

Each `version N/step/` folder has one STEP file per printed part, plus `assembly_deployed.step`, which shows the whole drone assembled with its arms open. These are real solids with flat faces and true cylinders, not triangle meshes like the STL files.

## Open one part

1. **File → Open**, and set the file type to STEP.
2. Pick the part, for example `version 2/step/arm_A_x2.step`.
3. If asked to run Import Diagnostics, choose **Yes**. It should find no faults.
4. **File → Save As → SOLIDWORKS Part (.SLDPRT)**.

## Convert everything at once

`convert_step_to_sldprt.bas` turns every STEP file into a native SOLIDWORKS file and saves them in a new `version N/sldprt/` folder.

1. Set your default part template, if you have never done so: **Tools → Options → System Options → Default Templates**.
2. **Tools → Macro → New**, and save it as `convert.swp` in any folder.
3. In the editor that opens, delete the existing text, then paste in all of `convert_step_to_sldprt.bas`.
4. Press **F5**. Enter the folder if it isn't `D:\3d print`.

This macro has not been run, because SOLIDWORKS isn't installed on the PC that wrote it. If it stops with an error, use the manual steps above and send the error message to get it fixed.

## What you can edit

An imported part arrives as one solid with no feature history, so there are no sketches or dimensions to double-click.

| To do this | Use |
|---|---|
| Move a face or resize a part | **Insert → Face → Move** |
| Fill a hole or remove a detail | **Insert → Face → Delete**, with **Delete and Patch** |
| Add holes, cuts or bosses | Normal sketches and features, on top of the imported solid |
| Get an editable feature tree | **FeatureWorks** (**Insert → FeatureWorks → Recognize Features**), if your licence includes it |

## Which copy to edit

There are now two ways to change a part, and they don't update each other.

- **SOLIDWORKS:** best for changing one part by hand. Export your own STL from it for printing.
- **`generate_frame.py`:** best for changes that affect many parts, such as prop size or motor spacing. It re-runs the clearance and rule checks, but overwrites the STL and STEP files, not your `.SLDPRT` edits.

After editing a part in SOLIDWORKS, the script's checks no longer cover that part, so check its fit in a SOLIDWORKS assembly.

## Regenerating the STEP files

Run this after changing a `generate_frame.py` and re-running it, using a Python 3.12 environment:

```
uv venv --python 3.12 .cadenv
uv pip install --python .cadenv/Scripts/python.exe cadquery-ocp shapely numpy trimesh scipy
.cadenv/Scripts/python.exe export_step.py
```

It rebuilds each part from the same design code, then compares its volume and size with the matching STL, and reports FAIL for any part that differs.
