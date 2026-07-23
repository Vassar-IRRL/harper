# Finger print files

Printable finger links for HARPER’s 5-finger hands, plus shared TPU joint connectors.

## Naming convention

Finger links use `{digit}_{segment}.stl` — matching URDF link names without the side / `_link` suffix.

| Token | Meaning |
|-------|---------|
| `thb` | Thumb |
| `ind` | Index |
| `mid` | Middle |
| `rng` | Ring |
| `pnk` | Pinky |
| `mip` | Metacarpophalangeal (base) |
| `pip` | Proximal interphalangeal |
| `dip` | Distal interphalangeal |
| `ip` | Thumb interphalangeal (tip) |
| `fng_` | Finger shared part |
| `plm_` | Palm interface (on shared connectors) |
| `tpu_joint_conn` | Flexible TPU joint connector |

### Shared connectors

| File | Role |
|------|------|
| `plm_mip_tpu_joint_conn.stl` | Palm ↔ MIP TPU joint |
| `fng_fng_tpu_joint_conn.stl` | Finger ↔ finger TPU joint |
