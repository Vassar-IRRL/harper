# HARPER Project

[![License](https://img.shields.io/github/license/smpdl/harper)](LICENSE)
[![Version](https://img.shields.io/badge/version-3.1-blue)](https://github.com/smpdl/harper)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/)
[![Documentation](https://img.shields.io/badge/docs-GitHub%20Pages-green)](https://paudelsamip.com.np/harper/)

This repository hosts the open-source hardware and software artifacts from the HARPER Project. The documentation for the project is hosted at [paudelsamip.com.np/harper](https://paudelsamip.com.np/harper/).

## Getting Started

This repository is organized into separate folders for hardware, software, documentation, and simulation. Below is what each top-level directory contains.

| Folder | Description |
|--------|-------------|
| [`harper_description/`](harper_description/) | ROS 2 package with the robot model: URDF/xacro, meshes, launch files, and RViz config. Start here to visualize the arm. *(WIP)* |
| [`harper_control/`](harper_control/) | Low-level control software for driving motors and reading joint feedback. *(WIP)* |
| [`harper_motion/`](harper_motion/) | Motion planning, trajectories, and higher-level arm commands. *(WIP)*|
| [`harper_simulation/`](harper_simulation/) | Simulation environments for testing without hardware. *(WIP)* |
| [`harper_print_files/`](harper_print_files/) | 3D-printable files for fabricating the arm's mechanical parts. *(WIP)* |
| [`harper_tests/`](harper_tests/) | Integration and regression tests across packages. *(WIP)* |
| [`docs/`](docs/) | Docusaurus site source: builds the project documentation hosted at [paudelsamip.com.np/harper](https://paudelsamip.com.np/harper/). |




