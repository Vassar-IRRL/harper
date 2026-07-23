# HARPER Print Files

3D-printable design files for fabricating HARPER's mechanical parts.

HARPER is a low-cost, 3D-printed dual-arm upper-body platform. This folder holds the printable hardware sources used to build the arm assemblies (shoulder through hand). 

## Left vs right

The left and right arms (and hands) are **mirrors of each other**. Segment folders ship **one side only**; you do **not** get separate left/right STLs.

For a full dual-arm build, print **two of each part** (unless a part README says otherwise).

## Layout

Parts are grouped by arm segment. Every folder has a README with an intro and naming guide.

| Folder | Contents |
|--------|----------|
| [`shoulder/`](shoulder/) | Shoulder joint housings and links |
| [`elbow/`](elbow/) | Elbow joint parts |
| [`forearm/`](forearm/) | Forearm roll / link parts |
| [`wrist/`](wrist/) | Wrist joint parts |
| [`palm/`](palm/) | Palm / hand base |
| [`fingers/`](fingers/) | Finger links and tips |
| [`assembly/`](assembly/) | Full-assembly CAD (STEP), not for printing |

## Bill of materials

Purchased parts and quantities are listed in the [Bill of Materials](BOM.md)

## Related packages

- [`harper_description/`](../harper_description/) — URDF/xacro and visualization meshes for the assembled robot
- [Project docs](https://vassar-irrl.github.io/harper/) — printing, assembly, and software guides

## License

Hardware designs in this folder are licensed under the [CERN Open Hardware Licence Version 2 – Strongly Reciprocal (CERN-OHL-S)](LICENSE). See the [repository README](../README.md#license) for how software and hardware licenses are split in this project.
