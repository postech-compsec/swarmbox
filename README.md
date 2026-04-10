# SwarmBox

![FSE 2026 Accepted](https://img.shields.io/badge/FSE_2026-Accepted-success?style=flat-square)
![License: MIT](https://img.shields.io/badge/License-MIT-blue?style=flat-square)
![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04-orange?style=flat-square)
![ROS 2](https://img.shields.io/badge/ROS_2-Humble-lightgrey?style=flat-square)

> [!WARNING]
> Please be advised: SwarmBox is currently in its early stages of development, and there might be some rough edges.
> We are actively working on improving the framework and will be updating the repository with more features, documentation, and examples in the near future.

**SwarmBox** is a plug-and-play drone swarm framework for streamlined development and comprehensive analysis. It decouples high-level swarm logic from low-level flight control and provides a swarm-level integrated analyzer to facilitate debugging and reproducible experimentation.

[![SwarmBox Demo](docs/assets/swarmbox_demo.gif)](https://youtube.com/playlist?list=PLblIEJCjwr_9GblkuCmMhJYYMpxl0o2AT&si=M_tvInOar4bzts4j)
[**Watch the full demo video playlist on YouTube**](https://youtube.com/playlist?list=PLblIEJCjwr_9GblkuCmMhJYYMpxl0o2AT&si=M_tvInOar4bzts4j)

## Documentation
For quick setup instructions, please refer to the [Setup Guide](docs/SETUP.md). 

For detailed documentation, including architecture overview, API reference, and usage examples, will be available soon. Please stay tuned for updates!

## About the Paper & Citation (To Appear)

This repository contains the source code and artifact for the following paper accepted 
at the **ACM International Conference on the Foundations of Software Engineering (FSE 2026)**.

> **SwarmBox: A Plug-and-Play Drone Swarm Framework for Streamlined Development and Comprehensive Analysis** 
>
> Minki Lee, Seojin Lee, and Seulbae Kim (To Appear)

If you use SwarmBox in your research, please cite our paper:

```tex
% Will be updated upon publication.
```

## Open Source License

As the goal of this project is to facilitate the research and development of drone swarm systems, 
SwarmBox is an open-source project licensed under the MIT License. See the [LICENSE](LICENSE) file for more details. 

Please feel free to use, modify, and distribute this software in accordance with the terms of the MIT License.

This project contains the following open-source components:
- **`PX4-Autopilot`**: BSD 3-Clause License (see [PX4-Autopilot/LICENSE](PX4-Autopilot/LICENSE))
- **`px4_msgs`**: BSD 3-Clause License (see [swarmbox_ws/src/px4_msgs/LICENSE](swarmbox_ws/src/px4_msgs/LICENSE))

## Zenodo Artifact
The fully packaged dataset and integrated execution environment for reproducing the results in our FSE 2026 paper are available on [10.5281/zenodo.19344308](https://doi.org/10.5281/zenodo.19344308). 

## Upcoming Features
> [!NOTE]
> The following features are currently under development or considered for future releases. We will update the repository with these features as they become available.

- [In-Progress]
    - Support for multi-layered DDS architecture for hierarchical swarm systems
- [Planned]
    - Hardware setup guide and description for real-world drone swarm deployment
    - Support for Recent ROS 2 Distributions and Ubuntu 24.04 LTS
    - Support for ArduPilot for broader hardware compatibility
- [Exploring]
    - Support for zenoh middleware for better performance and scalability
