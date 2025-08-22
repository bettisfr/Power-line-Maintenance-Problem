# C++ Code for the Drone–Truck Multi-Delivery Problem

![C++](https://img.shields.io/badge/-C++-blue?logo=cplusplus)
![License](https://img.shields.io/badge/license-MIT-green)

## Overview

This repository provides the C++ implementation of the algorithms and experiments described in our paper:

> **"Optimizing Drone–Truck Collaboration for Power Line Maintenance with Energy and Capacity Constraints"**,  
> submitted to **ACM Transactions on Sensor Networks (TOSN), 2025**.

---

### Problem Description

We study a coordinated delivery problem where a truck transports a drone to deliver tools and equipment to pylons on high-voltage power lines.  
Each **delivery** is characterized by:
- a target pylon,
- package weight,
- urgency-based profit,
- feasible launch and rendezvous points on the truck’s route.

The drone faces **energy and payload constraints**, may serve **multiple deliveries per mission**, and can execute multiple missions before recharging.  
The goal is to **maximize the total profit** of executed deliveries while ensuring feasibility.

We formalize this as the **Drone–Truck Multi-Delivery Problem (DTMDP)** and prove its NP-hardness. Our contributions include:

- A **two-phase solution approach**:
  - **Mission generation** via dynamic programming, constructing feasible multi-delivery missions.
  - **Mission selection** formulated as an **Integer Linear Programming (ILP)** problem.
- **Variants** considering unitary vs. arbitrary package weights and additional energy costs for package dropping.
- **Algorithms** with polynomial or pseudo-polynomial complexity for mission generation, contrasted with NP-hard mission selection.
- **Extensive experiments** showing that dynamic programming drastically reduces the number of candidate missions, enabling optimal solutions for instances with up to 80 deliveries and efficient suboptimal solutions for larger inputs.

---

## Installation

Clone the repository and build the project:

```bash
git clone https://github.com/bettisfr/Power-line-Maintenance-Problem
cd Power-line-Maintenance-Problem
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
./build/dcoss
```

---

## Contact

For questions, feedback, or collaboration opportunities:

- **Francesco Betti Sorbelli**: [francesco.bettisorbelli@unipg.it](mailto:francesco.bettisorbelli@unipg.it)  
- **Sajjad Ghobadi**: [sajjad.ghobadibabi@unipg.it](mailto:sajjad.ghobadibabi@unipg.it)  
- **Lorenzo Palazzetti**: [lorenzo.palazzetti@unipg.it](mailto:lorenzo.palazzetti@unipg.it)  
- **Cristina M. Pinotti**: [cristina.pinotti@unipg.it](mailto:cristina.pinotti@unipg.it)  
