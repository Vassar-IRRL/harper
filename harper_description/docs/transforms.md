# Transformation Matrices - harper_arm

Homogeneous transformation matrices between consecutive frames.
Convention: URDF RPY (XYZ extrinsic / ZYX intrinsic).

## Notation

### Frames

| Index | Link |
|-------|------|
| $L_{0}$ | base_link |
| $L_{1}$ | ubracket_link |
| $L_{2}$ | abduction_link |
| $L_{3}$ | shoulder_rot_link |
| $L_{4}$ | elbow_forearm_link |
| $L_{5}$ | forearm_front_link |

### Joint Variables

| Variable | Joint | Type | From | To |
|----------|-------|------|------|----|
| $q_{1}$ | shoulder_flexion_joint | revolute (rad) | $L_{0}$ | $L_{1}$ |
| $q_{2}$ | shoulder_abduction_joint | revolute (rad) | $L_{1}$ | $L_{2}$ |
| $q_{3}$ | shoulder_rotation_joint | continuous (rad) | $L_{2}$ | $L_{3}$ |
| $q_{4}$ | elbow_flexion_joint | revolute (rad) | $L_{3}$ | $L_{4}$ |
| $q_{5}$ | forearm_rotation_joint | continuous (rad) | $L_{4}$ | $L_{5}$ |

Shorthand: $c_i = \cos(q_i)$, $s_i = \sin(q_i)$

### Kinematic Tree

```
L0: base_link
  +-- [revolute] shoulder_flexion_joint (q1)
      L1: ubracket_link
        +-- [revolute] shoulder_abduction_joint (q2)
            L2: abduction_link
              +-- [continuous] shoulder_rotation_joint (q3)
                  L3: shoulder_rot_link
                    +-- [revolute] elbow_flexion_joint (q4)
                        L4: elbow_forearm_link
                          +-- [continuous] forearm_rotation_joint (q5)
                              L5: forearm_front_link
```

## Transforms

## shoulder_flexion_joint

$L_{0}$ **base_link** -> $L_{1}$ **ubracket_link** (revolute)
  Variable: $q_{1}$

- **origin xyz**: (0.04485, -0.007541, 0.039906) m
- **origin rpy**: (-0.261799, 0, 0) rad
- **axis**: (0, 1, 0)
- **limits**: [-3.141593, 1.570796] rad ([-180deg, 90deg])

### Local Transform

$T^{0}_{1}(q_{1}) = T_{fixed} \cdot R_{axis}(q_{1})$ where:

$$
T_{fixed} = \begin{bmatrix}
1 & 0 & 0 & 0.04485 \\
0 & 0.965926 & 0.258819 & -0.007541 \\
0 & -0.258819 & 0.965926 & 0.039906 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{1}) = \begin{bmatrix}
c_{1} & 0 & s_{1} & 0 \\
0 & 1 & 0 & 0 \\
-s_{1} & 0 & c_{1} & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## shoulder_abduction_joint

$L_{1}$ **ubracket_link** -> $L_{2}$ **abduction_link** (revolute)
  Variable: $q_{2}$

- **origin xyz**: (-0.0347, -0.0368, 0) m
- **origin rpy**: (0, 0, 0) rad
- **axis**: (-1, 0, 0)
- **limits**: [-0.261799, 3.141593] rad ([-15deg, 180deg])

### Local Transform

$$
T^{1}_{2}(q_{2}) = \begin{bmatrix}
1 & 0 & 0 & -0.0347 \\
0 & c_{2} & s_{2} & -0.0368 \\
0 & -s_{2} & c_{2} & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## shoulder_rotation_joint

$L_{2}$ **abduction_link** -> $L_{3}$ **shoulder_rot_link** (continuous)
  Variable: $q_{3}$

- **origin xyz**: (-0.01025, 0.054163, -0.035894) m
- **origin rpy**: (0.261799, 0, 0) rad
- **axis**: (0, 0.258819, 0.965926)

### Local Transform

$T^{2}_{3}(q_{3}) = T_{fixed} \cdot R_{axis}(q_{3})$ where:

$$
T_{fixed} = \begin{bmatrix}
1 & 0 & 0 & -0.01025 \\
0 & 0.965926 & -0.258819 & 0.054163 \\
0 & 0.258819 & 0.965926 & -0.035894 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{3}) = \begin{bmatrix}
c_{3} & -s_{3} & 0 & 0 \\
s_{3} & c_{3} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## elbow_flexion_joint

$L_{3}$ **shoulder_rot_link** -> $L_{4}$ **elbow_forearm_link** (revolute)
  Variable: $q_{4}$

- **origin xyz**: (0.089014, -0.119601, -0.186783) m
- **origin rpy**: (3.141593, 0, -3.141593) rad
- **axis**: (1, 0, 0)
- **limits**: [-1.396263, 0.610865] rad ([-80deg, 35deg])

### Local Transform

$T^{3}_{4}(q_{4}) = T_{fixed} \cdot R_{axis}(q_{4})$ where:

$$
T_{fixed} = \begin{bmatrix}
-1 & 0 & 0 & 0.089014 \\
0 & 1 & 0 & -0.119601 \\
0 & 0 & -1 & -0.186783 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{4}) = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & c_{4} & -s_{4} & 0 \\
0 & s_{4} & c_{4} & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## forearm_rotation_joint

$L_{4}$ **elbow_forearm_link** -> $L_{5}$ **forearm_front_link** (continuous)
  Variable: $q_{5}$

- **origin xyz**: (0.041862, 0.000337, 0.159713) m
- **origin rpy**: (-2.102137, 1.570796, 0) rad
- **axis**: (-1, 0, 0)

### Local Transform

$T^{4}_{5}(q_{5}) = T_{fixed} \cdot R_{axis}(q_{5})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & -0.862129 & -0.506689 & 0.041862 \\
0 & -0.506689 & 0.862129 & 0.000337 \\
-1 & 0 & 0 & 0.159713 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{5}) = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & c_{5} & s_{5} & 0 \\
0 & -s_{5} & c_{5} & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Global Transform Chains

Transform from root $L_0$ to any link, as product of local transforms along the kinematic chain.

$$T^{0}_{2} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2})\quad (L_0 \to L_{2}: \text{abduction_link})$$

$$T^{0}_{3} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3}(q_{3})\quad (L_0 \to L_{3}: \text{shoulder_rot_link})$$

$$T^{0}_{4} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3}(q_{3}) \cdot T^{3}_{4}(q_{4})\quad (L_0 \to L_{4}: \text{elbow_forearm_link})$$

$$T^{0}_{5} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3}(q_{3}) \cdot T^{3}_{4}(q_{4}) \cdot T^{4}_{5}(q_{5})\quad (L_0 \to L_{5}: \text{forearm_front_link})$$

