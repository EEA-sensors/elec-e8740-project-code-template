# ELEC-E8740 Basics of Sensor Fusion: project code

Starting point for the project of ELEC-E8740 Basics of Sensor Fusion (Aalto University, autumn 2026), done with the TurtleBot 4. The project guide on the course page describes the robot, the tasks and the data. This repository has no solutions on purpose: it only has empty starting files that suggest how to organise your code.

This is only a suggestion. If you prefer to organise your code differently, feel free to ignore these files.

## Getting started

1. Install Python 3 and the packages:

   ```
   pip install -r requirements.txt
   ```

2. Download your datasets from the course page and unzip them here. Each zip
   contains a folder `data/` with one subfolder per task. `data/description.txt`
   lists the files, and the appendix "Data files" of the project guide explains
   their columns.

## Suggested structure

The tasks build on each other. Part I gives you the numbers that the models of
Part II need, and Task 7 combines the models of Tasks 5 and 6:

```
Task 1  noise, gyro bias ────────────────┐
Task 3  focal length ──► camera model ───┼──► Task 5  localization ──┐
Task 4  wheel encoders ──► motion model ─┴──► Task 6  dead reckoning ─┴──► Task 7  EKF
```

So rather than writing one script per task, write each piece once and reuse it.
The starting files have the empty functions:

```
calibration.py    your Part I results (focal length, gyro bias, encoder gain, noise variances)
models.py         motion model and measurement model, each with its Jacobian
estimators.py     Gauss-Newton, dead reckoning, EKF
...               your own short scripts or notebooks per task that use these and make the figures
```

## License

Apache License 2.0, see `LICENSE`.
