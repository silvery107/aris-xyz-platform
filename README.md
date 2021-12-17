# Aris XYZ Platform


## Commands

| CMD | Description |
|:-:|:-|
|W, A, S, D|Move a small distance in `XY`|
|1, 2, 3, 4|Move `XY` to predefined grid points|
|C|Lift gripper to initial `Z` position|
|F|Drop down gripper in `Z`|
|R|Return gripper to zero `XYZ` position|
|\<space\>|Pick and place all in one|

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

## TODO
- [x] `WASD` command test passed
- [x] `F` command test passed
- [ ] `1,2,3,4` command test failed
- [ ] `C` command test failed
- [ ] `R` command test failed
