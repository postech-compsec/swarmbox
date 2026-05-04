---
layout: default
title: Getting Started
nav_order: 2
permalink: /setup/
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
Please refer to the official ROS 2 Jazzy installation guide for Ubuntu 22.04: [ROS 2 Jazzy Installation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

#### B. MicroXRCE-DDS
Currently, SwarmBox only supports MicroXRCE-DDS as the underlying middleware for communication between swarm components.
Please follow the instructions in the MicroXRCE-DDS documentation to install it on your system: [MicroXRCE-DDS Installation](https://docs.px4.io/v1.16/en/middleware/uxrce_dds)

> A [known build failure](https://github.com/PX4/PX4-Autopilot/issues/24477#issuecomment-3252813682) occurs in the MicroXRCE-DDS Make procedure 
> due to the deprecation of older FastDDS versions.
> To fix this, please update the `_fastdds_version` and `_fastdds_tag` variables as follows:
{: .warning }

```diff
--- a/CMakeLists.txt
+++ b/CMakeLists.txt
@@ -95,8 +95,8 @@ if(UAGENT_FAST_PROFILE)
     if(UAGENT_USE_SYSTEM_FASTDDS)
         set(_fastdds_version 2)
     else()
-        set(_fastdds_version 2.12)
-        set(_fastdds_tag 2.12.x)
+        set(_fastdds_version 2.13)
+        set(_fastdds_tag 2.13.x)
         set(_foonathan_memory_tag v0.7-3) # This tag should be updated every time it gets updated in foonathan_memory_vendor eProsima's package
     endif()
     list(APPEND _deps "fastrtps\;${_fastdds_version}")
```
{: .warning }


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

Please note that PX4-Autopilot is a submodule of the SwarmBox repository, so you don't need to clone it separately. 
Once the installation is complete, you can try the following command to verify that everything is set up correctly:

```bash
./scripts/functionality_check.sh
```
