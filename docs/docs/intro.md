---
sidebar_position: 1
---

# Introduction

**HARPER** (**H**umanoid **A**utonomous **R**obotic **P**latform for **E**xperimental **R**esearch) is an open-source humanoid platform for research and education. This page offers a short introduction of where the project comes from, why it exists, how we think about design, and where to go next in the docs.

## History

HARPER began at the [Interdisciplinary Robotics Research Lab (IRRL)](https://vassar-irrl.github.io) at Vassar College, led primarily by [Prof. Ken Livingston](https://www.vassar.edu/faculty/livingst) in the Department of Cognitive Science. Students and other members of the Vassar community have contributed throughout its development.

The work initially started with [InMoov](https://inmoov.fr/), Gael Langevin’s open-source, 3D-printed humanoid. It was an important early resource for learning to print and assemble a full-size form factor. Early student work with that platform is shown in [this video](https://vimeo.com/738351810/d240d6e5fd?fl=pl&fe=cm). As the lab’s goals moved toward research use: repeatable assembly, serviceability, and more precise, load-bearing control, the team decided a purpose-built design would fit those needs better.

That redesign became HARPER. Since then, the project has given Vassar students a lasting way to learn robotics by building and refining a real humanoid platform.

## Why does HARPER exist?

Humanoid robotics is an active field with many commercial and research platforms. Many of them are expensive, closed, or difficult to modify, which puts them out of reach for labs and students who want to run experiments or learn by building. HARPER was created to close that gap, with two main aims:

### Education

HARPER is meant to be a practical teaching platform. Students can print parts, assemble the robot, run it in simulation, and write control code without needing a large budget. Because the stack is open, coursework and student projects can inspect and change both hardware and software.

### Access

Commercial humanoids are often beyond what smaller labs and hobbyists can afford. HARPER keeps cost down with a 3D-printed structure and widely available Dynamixel servos, and releases hardware and software under open licenses so others can build, fork, and extend the platform.

## Philosophy

Many people have shaped HARPER over the years. Even so, the design aims to follow a small set of principles that guide how parts are made, how the software is structured, and what we optimize for.

### Open stack end to end

HARPER is open by design. Hardware and software are both first-class artifacts: print files and mechanical designs under CERN-OHL-S v2, and software and documentation under Apache 2.0. Anyone should be able to fabricate the robot, run the stack, and look through every layer without meeting a proprietary black box in the middle.


### Extensible and modular design

Mechanically, it is built in segments with a consistent connection pattern between links. A joint or link can be reprinted and replaced without redesigning the whole robot. Arms are mirrored, so the same segment design serves left and right. The goal is that people can mix, match, and iterate on parts as their needs change.

### Commodity actuation

HARPER does not rely on custom motor controllers or exotic actuators. Motion comes from Dynamixel servos on standard buses (for example U2D2 adapters), with configuration in open YAML rather than closed vendor tools alone.

### Value capture

Designing a humanoid with cost in mind always involves tradeoffs. Printable parts and off-the-shelf servos will not match every feature of an expensive commercial platform, and chasing that standard often produces something that is neither affordable nor satisfying to use.

HARPER’s guiding idea is value capture: get as much useful capability as we can from tools and parts that are already within reach: standard printers, commodity Dynamixels, open ROS 2 tooling, and designs others can rebuild. When we have to choose, we prefer the option that gives more research, teaching, or iteration value from those available pieces over the option that looks impressive on paper but depends on scarce or closed resources.

In practice, that means living with limits in materials, degrees of freedom, or polish when the alternative would put the platform out of reach, and putting effort where affordable parts and open software can still deliver real experimental and educational return.

