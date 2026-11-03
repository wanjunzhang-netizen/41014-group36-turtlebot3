# Validation results

Results from the single-camera simulation.

## Task tests

All 10 development trials completed without a collision and with a real wheel contact inside the 0.20 m charging circle. A pass also requires true docking target error below 0.04 m and true heading error below 8 degrees. These are evaluation checks, not ground-truth inputs to the controller.

|Seed|LiDAR noise (m)|Camera loss (s)|Time (s)|Position RMSE (m)|Dock error (m)|Heading error (degrees)|Pass|
|---|---|---|---|---|---|---|---|
|0|0.006|0.0|67.15|0.02604|0.01089|4.57|Yes|
|1|0.006|0.0|76.25|0.02250|0.01098|4.58|Yes|
|2|0.006|0.0|33.75|0.00184|0.01082|3.53|Yes|
|3|0.006|0.0|162.75|0.01185|0.01108|4.66|Yes|
|4|0.006|0.0|36.95|0.02887|0.01088|1.37|Yes|
|5|0.006|0.0|314.65|0.00234|0.01090|4.79|Yes|
|0|0.012|0.0|67.15|0.02604|0.01089|4.57|Yes|
|0|0.006|4.0|67.30|0.02469|0.01116|4.82|Yes|
|1|0.012|0.0|74.10|0.02355|0.01079|4.70|Yes|
|1|0.006|4.0|74.55|0.02281|0.01091|4.89|Yes|

Position RMSE starts after the first valid camera initialization. The controlled comparisons keep the same seed and change LiDAR noise or remove camera measurements for four seconds. Camera loss starts five seconds after initialization. A different path can change later noise samples, so a faster disturbed run does not show that noise helps.

Seed 0 has a box 30% along the start-to-station line. An earlier candidate with a larger box halfway along the line timed out after 360 seconds without a collision. That candidate was not used. The current results are selected development trials, not proof that every random room will work.

## Model and sensor checks

- 13 of 15 rendered camera views were accepted. At 0.16 m and yaw +/-15 degrees, the marker was clipped and detection failed.
- Accepted maximum camera range error was about 0.00608 m; maximum bearing error was about 0.0542 degrees.
- This is validation against known simulated geometry, not physical camera calibration.

## Reproduce and inspect

```powershell
python -m unittest tests.check_walls tests.check_alignment tests.check_camera tests.check_discovery tests.check_assignment_alignment
python tools/analyze_model.py
python tools/validate_camera_calibration.py
python tools/validate_discovery.py
```

Evidence: evidence/validation_summary.json and calibration/monocular_validation.json.
