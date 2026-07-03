---
layout: default
title: Documentation
nav_order: 3
has_children: true
permalink: /docs/
has_toc: false
---

# Documentation for SwarmBox

This documentation is designed for researchers and developers who want to use SwarmBox as a practical platform for real swarm-drone development workflows.

The focus is on:
- building and running swarm scenarios quickly,
- understanding where to customize behavior,
- operating simulation and real-world deployments safely,
- and troubleshooting common integration issues.

The focus is not on reproducing paper artifacts. If you are looking for publication artifacts, see the project main page and Zenodo links.

## Who This Guide Is For
- Researchers prototyping and evaluating swarm system components.
- Developers integrating custom mission logic into a ROS 2 and PX4-based stack.

> Currently, SwarmBox supports only the PX4 flight stack, but we plan to carefully craft the modular joint architecture to support other robotic stacks in the future.
> {: .note}

## Table of Contents

We recommend reading the documentation in order, but you can also jump to specific sections as needed. The following is a suggested reading order:

> The documentation is still under development, and some sections are incomplete. Sections marked with a 🚧 are works in progress and will be updated soon.
> {: .new}

1. [🚧 Getting Started]<!-- ({% link pages/documentation/getting-started.md %}) -->  
2. [🚧 Architecture Overview]<!-- ({% link pages/documentation/architecture.md %}) -->  
3. [🚧 Mission Configuration]<!-- ({% link pages/documentation/mission.md %}) -->  
4. [Network Configuration]({% link pages/documentation/network.md %})  
5. [🚧 Implementing Custom Logic]<!-- ({% link pages/documentation/implementation.md %}) -->  
6. [🚧 Execution]<!-- ({% link pages/documentation/execution/index.md %}) -->  
    a. [🚧 SITL Simulation]<!-- ({% link pages/documentation/execution/SITL-simulation.md %}) -->  
    b. [🚧 Hardware Deployment]<!-- ({% link pages/documentation/execution/hardware-deployment.md %}) -->  
7. [🚧 Troubleshooting]<!-- ({% link pages/documentation/troubleshooting.md %}) -->  

## What You Will Learn
- How SwarmBox separates swarm-level logic from low-level flight control.
- How to configure swarm scenarios using mission config files.
- How to run and manage multi-drone SITL sessions.
- How to prepare for simulation-to-real transitions.
- How to avoid common setup and networking pitfalls.

## Start Here
If you are new to the project, begin with [Getting Started]({% link pages/documentation/getting-started.md %}) and aim to complete one successful SITL run first.

