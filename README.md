# HARPER Project

[![License](https://img.shields.io/github/license/Vassar-IRRL/harper)](LICENSE)
[![Version](https://img.shields.io/badge/version-3.1-blue)](https://github.com/Vassar-IRRL/harper)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/)
[![Documentation](https://img.shields.io/badge/docs-GitHub%20Pages-green)](https://paudelsamip.com.np/harper/)

This repository hosts the open-source hardware and software artifacts from the HARPER Project. The documentation for the project is hosted at [paudelsamip.com.np/harper](https://paudelsamip.com.np/harper/).

## About the Project
HARPER stands for Humanoid Autonomous Robotic Platform for Experimental Research. HARPER was started as a long-term project in the Interdisciplinary Robotics Research Lab (IRRL) and the Cognitive Science Department at Vassar College. The main objective of this project is to create a low-cost 3D printed humanoid robot platform for researchers in behavioral and cognitive science and for hobbyist use.

## Getting Started

This repository is organized into separate folders for hardware, software, documentation, and simulation. Below is what each top-level directory contains. Each folder also contains detailed README relevant for the specific package.  

| Folder | Description |
|--------|-------------|
| [`harper_description/`](harper_description/) | ROS 2 package with the robot model: URDF/xacro, meshes, launch files, and RViz config. Start here to visualize the arm. *(WIP)* |
| [`harper_control/`](harper_control/) | Low-level control software for driving motors and reading joint feedback. *(WIP)* |
| [`harper_motion/`](harper_motion/) | Motion planning, trajectories, and higher-level arm commands. *(WIP)*|
| [`harper_simulation/`](harper_simulation/) | Simulation environments for testing without hardware. *(WIP)* |
| [`harper_print_files/`](harper_print_files/) | 3D-printable files for fabricating the arm's mechanical parts. *(WIP)* |
| [`harper_tests/`](harper_tests/) | Integration and regression tests across packages. *(WIP)* |
| [`docs/`](docs/) | Docusaurus site source: builds the project documentation hosted at [paudelsamip.com.np/harper](https://paudelsamip.com.np/harper/). |


## Contributing

Issues and pull requests are welcome. See [CONTRIBUTING](CONTRIBUTING) for how to get started, what we're looking for, and PR guidelines.


## License

This project is licensed under the [Apache License, Version 2.0](LICENSE).

You are free to use, modify, and distribute this software and documentation under the terms of the license. See the [LICENSE](LICENSE) file for the full text.


