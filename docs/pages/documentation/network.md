---
layout: default
title: Network Configuration
parent: Documentation
nav_order: 4
permalink: /docs/network/
has_toc: true
---
# Network Configuration

The communication between the components of SwarmBox is defaultly configured to use DDS for scalable and efficient communication.
However, DDS faces discovery storm issues when the number of nodes connected to DDS increases, which can lead to performance degradation and communication failures.
To address this issue, from `v0.3.0`, SwarmBox introduces a multi-domain DDS network configuration that separates the communication into two distinct domains: Global Network and Local Network.

## Multi-Domain Separation of Networks
SwarmBox utilizes the Data Distribution Service (DDS) for communication between the each components of the system.
Swarm system consists of three main components: Ground Control Station (**GCS**), Companion Computer (**CC**), and Flight Controller (**FC**).

For some cases, you can skip the CC layer and directly connect the GCS to the FC for simple and small-scale swarm systems, 
but since SwarmBox is designed for complex and sophisticated usages, SwarmBox incorporates the CC layer.

Therefore, the communication between the components can be divided into two main networks: Global Network and Local Network.

## Global Network: GCS -- CC
Global Network is the network connection between SwarmBox Nodes, including the `Drone` Node on the CC and the `Ground` Node on the GCS.
This network is responsible for the communication between SwarmBox Nodes, which can be ground-to-drone or drone-to-drone communication.
For scalable and flexible communication, the global network should be configured to use a single domain space, which allows dynamic and flexible communication architecture during runtime.

## Local Network: CC -- FC
Local Network is the network connection between the `Drone` Node on the CC and the FC (e.g., PX4 on Pixhawk).
This network is responsible for the communication between the CC and the FC, which is typically a one-to-one connection.
To optimize bandwidth and reduce communication overhead, the local network should be configured to use a separate domain space for each drone, which allows for more efficient and reliable communication between the CC and the FC.
The internal messages exchanged between the CC and the FC are typically high-frequency messages, such as sensor data and control commands, which require low latency and high reliability.
At the same time, these datas doesn't have to be broadcast to other drones; if needed, the data can be shared with other drones through the global network.

## How domains are configured
The domain configuration for the Global Network and Local Network is as follows:

### Rules
- Global Network (GCS <-> CC):
    - `ROS_DOMAIN_ID=0`
    - `dds_port=8888`
- Local Network (CC <-> FC):
    - `ROS_DOMAIN_ID=10+drone_id`
    - `dds_port=8888+10+drone_id`

In other words, all nodes that should participate in swarm-wide communication use the same global domain,
while each drone gets its own local domain for CC-FC traffic.

### Example Mapping

| Drone ID | Global `ROS_DOMAIN_ID` | Global `dds_port` | Local `ROS_DOMAIN_ID` | Local `dds_port` |
|---:|---:|---:|---:|---:|
| 1 | 0 | 8888 | 11 | 8899 |
| 2 | 0 | 8888 | 12 | 8900 |
| 3 | 0 | 8888 | 13 | 8901 |

### Why this layout is used
- The global domain keeps cross-drone and ground-drone communication simple and discoverable.
- The per-drone local domain isolates high-rate CC-FC traffic, reducing unnecessary discovery and bandwidth usage.
- Local traffic can be confined to localhost or a dedicated interface, improving scalability in larger swarms.

### Notes
- Ensure each drone has a unique `drone_id`; otherwise, local domains and ports will collide. 
  The default execution of SITL will assign unique drone IDs automatically (from 0), but for real drones, you might have to assign unique IDs manually.
- Keep the global domain and global port consistent across all participants in the same swarm session.
- If you change base values (`0`, `8888`, or the `+10` offset), update all related launch and configuration files consistently.
