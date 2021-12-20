# Aris XYZ Platform


## Commands

| Key        | Description                          | Test |
|:----------:|:-------------------------------------|:----:|
| W, A, S, D | Move a small distance in `XY`        | Pass |
| 1, 2, 3, 4 | Move `XY` to predefined grid points  | Pass |
| C          | Lift gripper to initial `Z` position | Pass |
| F          | Drop down gripper in `Z`             | Pass |
| E          | Pick and place all in one            | Pass |
| R          | Return gripper to zero `XYZ` position| Fail |


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
## Example Command Flow
```
    c           // lift Z to initial pos and record this XYZ pos as (0,0)
    1/2/3/4     // move gripper to predefined point
    f           // drop Z
    e           // pick and place
    r           // return to (0,0)
```
