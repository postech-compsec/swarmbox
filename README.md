# SwarmBox

![FSE 2026 Accepted](https://img.shields.io/badge/FSE_2026-Accepted-success?style=flat-square)
![License: MIT](https://img.shields.io/badge/License-MIT-blue?style=flat-square)
![Ubuntu 24.04](https://img.shields.io/badge/Ubuntu-24.04-orange?style=flat-square)
![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-lightgrey?style=flat-square)


> [!WARNING]
> Please be advised: SwarmBox is currently in its early stages of development, and there might be some rough edges.
> We are actively working on improving the framework and will be updating the repository with more features, documentation, and examples in the near future.

**SwarmBox** is a plug-and-play drone swarm framework for streamlined development and comprehensive analysis. It decouples high-level swarm logic from low-level flight control and provides a swarm-level integrated analyzer to facilitate debugging and reproducible experimentation.


[![SwarmBox Demo](docs/assets/swarmbox_demo.gif)](https://youtube.com/playlist?list=PLblIEJCjwr_9GblkuCmMhJYYMpxl0o2AT&si=M_tvInOar4bzts4j)
[**Watch the full demo video playlist on YouTube**](https://youtube.com/playlist?list=PLblIEJCjwr_9GblkuCmMhJYYMpxl0o2AT&si=M_tvInOar4bzts4j)

## Project Homepage & Documentation

We host a dedicated project homepage to enhance the discoverability of SwarmBox and provide highly readable documentation.

- **Project Homepage:** [https://compsec.postech.ac.kr/swarmbox](https://compsec.postech.ac.kr/swarmbox)

Detailed documentation, including architecture overview, API reference, and usage examples, will be available soon.
Please stay tuned for updates!

## Paper & Citation

SwarmBox has been accepted for publication in the proceedings of
the **ACM International Conference on the Foundations of Software Engineering (FSE 2026)**.

> **SwarmBox: A Plug-and-Play Drone Swarm Framework for Streamlined Development and Comprehensive Analysis**  
> by Minki Lee, Seojin Lee, and Seulbae Kim

If you find SwarmBox useful for your research, please consider citing our paper ([pdf](https://compsec.postech.ac.kr/assets/publications/lee:swarmbox.pdf), [DOI](https://doi.org/10.1145/3808100)):

```bibtex
@article{lee2026swarmbox,
    author = {Lee, Minki and Lee, Seojin and Kim, Seulbae},
    title = "{SwarmBox}: A Plug-and-Play Drone Swarm Framework for Streamlined Development and Comprehensive Analysis",
    year = {2026},
    issue_date = {July 2026},
    publisher = {Association for Computing Machinery},
    address = {New York, NY, USA},
    volume = {3},
    number = {FSE},
    url = {https://doi.org/10.1145/3808100},
    doi = {10.1145/3808100},
    journal = {Proc. ACM Softw. Eng.},
    articleno = {FSE093},
    numpages = {22},
    keywords = {Framework, Swarm, Drone, Cyber-Physical Systems, Distributed CPS}
}
```

## Zenodo Artifact
The fully packaged dataset and integrated execution environment for reproducing the results in our FSE 2026 paper are available on [10.5281/zenodo.19344308](https://doi.org/10.5281/zenodo.19344308). 

## Open Source License

As the goal of this project is to facilitate the research and development of drone swarm systems, 
SwarmBox is an open-source project licensed under the MIT License. See the [LICENSE](LICENSE) file for more details. 

Please feel free to use, modify, and distribute this software in accordance with the terms of the MIT License.

This project contains the following open-source components:
- **`PX4-Autopilot`**: BSD 3-Clause License (fetched by `autopilot/setup-px4.sh`; see [PX4-Autopilot LICENSE](https://github.com/PX4/PX4-Autopilot/blob/main/LICENSE))
- **`px4_msgs`**: BSD 3-Clause License (see [swarmbox_ws/src/px4_msgs/LICENSE](swarmbox_ws/src/px4_msgs/LICENSE))
