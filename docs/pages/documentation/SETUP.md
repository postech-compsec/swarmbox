---
layout: default
title: Getting Started
parent: Documentation
nav_order: 1
permalink: /docs/getting-started/
has_toc: true
---

# How to Setup SwarmBox

This document provides step-by-step instructions for setting up the SwarmBox framework on your local machine. Please follow the instructions carefully to ensure a successful installation.

## System Requirements (Recommended)
- **Operating System**: Ubuntu 24.04 LTS
- **ROS 2 Distribution**: Jazzy
- **Python Version**: 3.12 or later

## Installation Steps

### 1. Install Dependencies
#### A. ROS 2 Jazzy
Please refer to the official ROS 2 Jazzy installation guide for Ubuntu 24.04: [ROS 2 Jazzy Installation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

#### B. MicroXRCE-DDS
Currently, SwarmBox only supports MicroXRCE-DDS as the underlying middleware for communication between swarm components.
Please follow the instructions in the MicroXRCE-DDS documentation to install it on your system: [MicroXRCE-DDS Installation](https://docs.px4.io/v1.17/en/middleware/uxrce_dds)


### 2. Clone SwarmBox Repository

You can clone the SwarmBox repository using the following command:

```bash
git clone https://github.com/postech-compsec/swarmbox.git --recursive
```

> **Notice for Zenodo Artifact / Ubuntu 22.04 Users**  \\
> If you are looking for the original artifact published on Zenodo (Ubuntu 22.04 / ROS 2 Humble / PX4 v1.15), 
> please refer to the SETUP.md of [v0.1.0](https://github.com/postech-compsec/swarmbox/blob/v0.1.0/docs/pages/SETUP.md).
{: .highlight }

### 3. Install SwarmBox and Dependencies
We provide a setup script to automate the installation of SwarmBox and its dependencies. Run the following command in the root directory of the cloned repository:

```bash
./scripts/setup.sh
```

This will also run `autopilot/setup-px4.sh`, which clones PX4-Autopilot (`release/1.17`) into `autopilot/PX4-Autopilot/` 
and applies the SwarmBox-specific patches from `autopilot/PX4-patches/`. 

PX4-Autopilot is intentionally not vendored as a submodule anymore; instead, it will be cloned during the setup process.
Once the installation is complete, you can try the following command to verify that everything is set up correctly:

```bash
./scripts/functionality_check.sh
```
