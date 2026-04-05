# SwarmBox

![FSE 2026 Accepted](https://img.shields.io/badge/FSE_2026-Accepted-success?style=flat-square)
![License: MIT](https://img.shields.io/badge/License-MIT-blue?style=flat-square)
![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04-orange?style=flat-square)
![ROS 2](https://img.shields.io/badge/ROS_2-Humble-lightgrey?style=flat-square)

**SwarmBox** is a plug-and-play drone swarm framework for streamlined development and comprehensive analysis. It decouples high-level swarm logic from low-level flight control and provides a swarm-level integrated analyzer to facilitate debugging and reproducible experimentation.

[![SwarmBox Demo](docs/assets/swarmbox_demo.gif)](https://youtube.com/playlist?list=PLblIEJCjwr_9GblkuCmMhJYYMpxl0o2AT&si=M_tvInOar4bzts4j)

<!-- 
## Documentation

For detailed usage instructions and architecture overview of SwarmBox, please refer to the following documents:

- [1. Getting Started](docs/1_GETTING_STARTED.md)
- [2. Quick Start Tutorial](docs/2_QUICK_START.md)
- [3. System Architecture](docs/3_ARCHITECTURE.md)
- [4. Integrated Analyzer](docs/4_ANALYZER.md)
- [5. Reproducing Paper Results (AE)](docs/5_ARTIFACT_EVALUATION.md) -->


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
