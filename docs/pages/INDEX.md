---
layout: default
title: SwarmBox
nav_order: 1
permalink: /
---

# SwarmBox 
**SwarmBox** is an open-source drone swarm framework that aims to **decouple high-level swarm logic** from low-level flight control and **provide a swarm-level integrated analyzer** to facilitate debugging and reproducible experimentation.

> **SwarmBox: A Plug-and-Play Drone Swarm Framework for Streamlined Development and Comprehensive Analysis** (To Appear).  
> Minki Lee, Seojin Lee, and Seulbae Kim.  
> In *Proceedings of the 34th ACM International Conference on the Foundations of Software Engineering ([FSE 2026](https://conf.researchr.org/home/fse-2026))*,  
> Montreal, Canada, July 2026. 

[<span class="material-symbols-outlined">code</span> GitHub](https://github.com/postech-compsec/swarmbox){: .btn .btn-outline }
[<span class="material-symbols-outlined">docs</span> Paper](https://compsec.postech.ac.kr/assets/publications/lee:swarmbox.pdf){: .btn .btn-outline }
[<span class="material-symbols-outlined">archive</span> Archive](https://doi.org/10.5281/zenodo.19344308){: .btn .btn-outline }
[<span class="material-symbols-outlined">slideshow</span> Videos](https://youtube.com/playlist?list=PLblIEJCjwr_9GblkuCmMhJYYMpxl0o2AT&si=M_tvInOar4bzts4j){: .btn .btn-outline }

> Please be advised: SwarmBox is currently in its early stages of development, and there might be some rough edges.
> We are actively working on improving the framework and will be updating the repository with more features, documentation, and examples in the near future.
{: .warning }

## Introduction
While drone swarms are emerging as a paradigm-shifting technology, progress in research is often constrained by a fragmented development ecosystem. Every new algorithm requires a bespoke testbed, introducing redundant engineering overhead and making results difficult to reproduce. As a remedy, we present SwarmBox, an open-source framework that provides a shared foundation for swarm robotics. 

![SwarmBox Overview](assets/overview.png)

By offering a plug-and-play architecture, a swarm-level integrated analyzer, and configurable experimentation environments, SwarmBox streamlines development and empowers reproducible, community-driven swarm research.

## Demo

<div style="width: 100%; aspect-ratio: 16/9;">
    <iframe width="100%" height="100%" src="https://www.youtube.com/embed/HuoiBodCa1g?si=RhoVFkt3eU25RAZR" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
</div>

[**Watch the full demo video playlist on YouTube**](https://youtube.com/playlist?list=PLblIEJCjwr_9GblkuCmMhJYYMpxl0o2AT&si=M_tvInOar4bzts4j)


## Deep Dive into SwarmBox
For quick setup instructions, please refer to the [Getting Started]({% link pages/SETUP.md %}).  
For detailed documentation, including architecture overview and usage examples, will be available soon. Please stay tuned for updates!

## Cite this work
If you find SwarmBox useful for your research, please consider citing our paper:
([pdf](https://compsec.postech.ac.kr/assets/publications/lee:swarmbox.pdf), [DOI](https://doi.org/10.1145/3808100)):

```bibtex
@article{lee2026swarmbox,
    author = {Lee, Minki and Lee, Seojin and Kim, Seulbae},
    title = "{SwarmBox}: A Plug-and-Play Drone Swarm Framework 
             for Streamlined Development and Comprehensive Analysis",
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

## Open Source License

As the goal of this project is to facilitate the research and development of drone swarm systems, 
SwarmBox is an open-source project licensed under the MIT License. 
Please feel free to use, modify, and distribute this software in accordance with the terms of the MIT License.

See the [LICENSE]({% link pages/LICENSE.md %}) for more details. 

## Zenodo Artifact
The fully packaged dataset and integrated execution environment for reproducing the results in our FSE 2026 paper 
are available on [10.5281/zenodo.19344308](https://doi.org/10.5281/zenodo.19344308). 

## Upcoming Features
> The following features are currently under development or considered for future releases. We will update the repository with these features as they become available.
{: .note }

- **In-Progress**
    - Support for multi-layered DDS architecture for hierarchical swarm systems
- **Planned**
    - Hardware setup guide and description for real-world drone swarm deployment
    - Support for Recent ROS 2 Distributions and Ubuntu 24.04 LTS
    - Support for ArduPilot for broader hardware compatibility
- **Exploring**
    - Support for zenoh middleware for better performance and scalability
