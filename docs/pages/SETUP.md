---
layout: default
title: Getting Started
nav_order: 2
permalink: /setup/
---

# How to Setup SwarmBox

This document provides step-by-step instructions for setting up the SwarmBox framework on your local machine. Please follow the instructions carefully to ensure a successful installation.

## System Requirements (Recommended)
- **Operating System**: Ubuntu 22.04 LTS
- **ROS 2 Distribution**: Humble
- **Python Version**: 3.10 or later

## Installation Steps

### 1. Install ROS 2 Humble
Please refer to the official ROS 2 Humble installation guide for Ubuntu 22.04: [ROS 2 Humble Installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

### 2. Clone SwarmBox Repository
#### A. Cloning from GitHub
If you are setting up SwarmBox from our public repository, 
you can clone the SwarmBox repository using the following command:

```bash
git clone https://github.com/postech-compsec/swarmbox.git
```

#### B. Cloning from Zenodo
If you have downloaded the SwarmBox artifact from Zenodo,
you already have the main source code. 
However, the heavy `PX4-Autopilot` submodule was not included in the Zenodo artifact due to size constraints.
You must manually clone our modified PX4 repository into the `repository/PX4-Autopilot` directory before proceeding:
```bash
# in repository/ directory,
git clone -b swarmbox/px4-v1.15 --single-branch --recursive https://github.com/postech-compsec/swarmbox-PX4.git PX4-Autopilot
# to prevent potential issues with tags for PX4 make, we recommend using the following command to add tag manually after cloning:
cd PX4-Autopilot
git tag v1.15.4
```

### 3. Install SwarmBox and Dependencies
We provide a setup script to automate the installation of SwarmBox and its dependencies. Run the following command in the root directory of the cloned repository:

```bash
./scripts/setup.sh
```

Please note that PX4-Autopilot is a submodule of the SwarmBox repository, so you don't need to clone it separately. 
Once the installation is complete, you can try the following command to verify that everything is set up correctly:

```bash
./scripts/functionality_check.sh
```
