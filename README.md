# Aris XYZ Platform


## Commands

| Key        | Description                          | Test |
|:----------:|:-------------------------------------|:----:|
| W, A, S, D | Move a small distance in `XY`        | Pass |
| 1, 2, 3, 4 | Move `XY` to predefined grid points  | Fail |
| C          | Lift gripper to initial `Z` position | Fail |
| F          | Drop down gripper in `Z`             | Pass |
| R          | Return gripper to zero `XYZ` position| Fail |
| E          | Pick and place all in one            | Fail |


## Conventions

```
_________
|   |   |
|  END  |
|   |   |
|_(0,0)_|
| 1 | 2 |
| 3 | 4 |
---------
x ^
  |__> y

```
