# Aris XYZ Platform

## Introduction

Course project of ME337 Advanced Actuation for Robots at SUSTech.

## Soft Acutator & Gripper
<!-- <p float="left">
<img src=images/finger_model_render.png width=200>
<img src=images/gripper_model_render.png width=300>
</p> -->
- Soft pneumatic linear actuator
<img src=images/acuator.png width=400>

- Soft linkage gripper
<img src=images/gripper.png width=400>

- MATLAB Similation
<img src="models/linkage_length_optimization/test.gif" width=200>


## 3-Axis Platform

https://user-images.githubusercontent.com/44640904/157364136-9e3fa75b-e46b-4bdc-a909-37e39be5fc04.mp4

- iHSS Integrated stepper servo motors
- Industrial personal computer
- EtherCat communication

<p float="left">
<img src=images/xyz_render.png width=200>
<!-- <img src=images/finger_model_render.png width=200>
<img src=images/gripper_model_render.png width=300> -->
<img src=images/xyz_final.jpg width=300>
</p>



## Trapezoidal Speed Planner
### Features
- Large distance, long time and high speed planning
- Optimal time and given time trapezoidal planning
- Adaptive trapezoidal to triangular velocity profile planning
- Time synchronization for 3-axis
### Simulation
- `<test/T_planner_plot.py>`

<img src=images/T_planner_test.png width=500>




## Commands

| Key        | Description                          |
|:----------:|:-------------------------------------|
| W, A, S, D | Move a small distance in `XY`        |
| 1,2,3,4,5  | Move `XY` to predefined grid points  |
| C          | Lift gripper to initial `Z` position |
| F          | Drop down gripper in `Z`             |
| G          | Lift up gripper in `Z`               |
| E          | Pick and place all in one            |
| R          | Return gripper to zero `XYZ` position|

### Conventions

```
_________
|   |   |
|  END  |
|   |   |
|_(0,0)_|
| 1 | 2 |
|   5   |
| 3 | 4 |
---------
x ^
  |__> y
```

### Example
```bash
c               # lift Z to initial pos and record this XYZ pos as (0,0)
1 / 2 / 3 / 4   # move gripper to predefined point
w / a / s / d   # fine tune XY position
f/g             # drop or lift Z
e               # pick and place
r               # return to (0,0)
```

## Dependencies

- [Aris 1.5.0](https://github.com/py0330/aris)
- EtherCat
- Real-time operating system (RTOS)
