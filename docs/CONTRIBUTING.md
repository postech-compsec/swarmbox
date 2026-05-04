---
layout: default
title: Contributing to SwarmBox
parent: Community Guides
nav_order: 2
# permalink: /community/contributing/
---

# Contributing to SwarmBox

First off, thank you for considering contributing to SwarmBox! 

As our framework is currently in the early stages of development, 
our comprehensive contributing guidelines are still evolving. 
For now, please refer to the basic processes outlined below. 
We welcome all kinds of contributions, including bug reports, feature requests, and code improvements.

*NOTE: By contributing to SwarmBox, you agree that your contributions will be licensed under its MIT License.*

## How to Report an Issue

- **Bug Reports & Feature Requests:** 
    Please use the GitHub Issues tab to report bugs or suggest new features.
- **Search First:** 
    Before creating a new issue, please check if a similar issue already exists to avoid duplicates.
- **Security Vulnerabilities:** 
    Do **not** report security vulnerabilities via public issues. 
    Please refer to our [Security Policy](../SECURITY) for private reporting instructions.

## Pull Request Process

We actively welcome your Pull Requests! To submit code changes:

1. **Fork** the repository and create your working branch from `main` (e.g., `feature/awesome-new-feature` or `fix/issue-123`).
2. Make your changes and commit them with clear, descriptive messages.
3. Open a **Pull Request (PR)** against the `main` branch.
4. Provide a clear description of the problem you've solved or the feature you've added.
5. **Code Review:** Please note that all PRs require at least one approval from a project maintainer before they can be merged.

### Branch Naming Convention

To keep our repository organized, please use the following branch naming conventions when creating your working branch:

- `feature/feature-name` (for new features or enhancements)
- `fix/issue-#` or `fix/bug-name` (for bug fixes)
- `docs/document-name` (for documentation updates)
- `chore/task-name` (for maintenance tasks, such as updating dependencies)

## Policy on AI-Generated or Automated Contributions

While we acknowledge the utility of AI coding assistants (e.g., GitHub Copilot, ChatGPT) in development, 
**you are strictly responsible for the accuracy, quality, and relevance of any content you submit.**

Please be aware of the following policies:
- **Quality Control:** 
    You must thoroughly review and test any generated code or text before submitting a Pull Request or Issue.
- **Spam Prevention:** 
    Submitting excessive, low-quality, or hallucinated AI-generated Issues or Pull Requests 
    that disrupt the normal operation and maintainability of this project is strictly prohibited.
- **Consequences:** 
    We reserve the right to immediately close such automated spam without review. 
    Repeated violations may result in restrictions or a permanent ban from contributing to the repository.

## Original Evaluation & Development Environment

Currently, the framework has only been fully evaluated in our original development environment.  
While we plan to test and support broader OS versions and setups in the future, 
please refer to the following hardware and software stack used for our ongoing development:

- **Workstation**: 
    - **Hardware**: AMD 9800X3D, 64GB RAM
    - **OS and ROS 2**: 
        - 24.04 LTS with ROS 2 Jazzy
        - 22.04 LTS with ROS 2 Humble (**DEPRECATED**, for original artifact on Zenodo)
- **Physical Drones (NOT REQUIRED FOR VM EXECUTION)**: 
    - **Drone**: Holybro X500 V2
    - **Flight Controller**: Pixhawk 6C
    - **Firmware**: PX4 v1.15.4 with minor modifications (see postech-compsec/swarmbox-PX4 repository for details)
    - **Companion Computer**: Raspberry Pi 3 Model B
    - **Companion Computer OS**: Ubuntu 22.04.5 LTS

Thank you for helping us improve SwarmBox!
