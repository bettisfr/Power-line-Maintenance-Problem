# C++ Code for Optimizing Single-Drone and Truck Collaboration for Last-Mile Delivery with Energy and Capacity Constraints

![C++](https://img.shields.io/badge/-C++-blue?logo=cplusplus)
![License](https://img.shields.io/badge/license-MIT-green)

## Overview

This repository contains the C++ implementation of the algorithms and experiments described in our paper, **"Optimizing Single-Drone and Truck Collaboration for Last-Mile Delivery with Energy and Capacity Constraints"**, which has been submitted for publication in **21st IEEE International Conference on Distributed Computing in Smart Systems and the Internet of Things (DCOSS-IoT 2025)**.

In this paper, we investigate the collaboration between a truck and a single drone in a last-mile package delivery scenario, introducing the Truck-drone Multi-delivery Problem (TMP). The truck follows a predefined route and transports the drone, which is capable of carrying multiple packages in a single flight. Each delivery is characterized by an energy cost, a capacity requirement, a profit representing its priority, and a delivery interval defined by two points on the truck’s route: the launch point, where the drone departs, and the rendezvous point, where the drone returns to the truck. The objective of the TMP is to schedule the drone’s flights to maximize the total profit while respecting the constraints of the drone’s energy capacity and payload limit. Conflicts between deliveries, arising when delivery intervals overlap, must be resolved unless the conflicting deliveries are combined into a single flight that satisfies the capacity and energy constraints. 

We show that TMP is NP-hard and propose an Integer Linear Programming (ILP) formulation to solve it optimally. We consider two scenarios: TMP with Unitary package weights (TMP-U), and TMP with Arbitrary package weights (TMP-A), making the problem significantly more challenging. For both scenarios, we develop tailored algorithms to address the challenges effectively. Finally, we evaluate the performance of our proposed algorithms on diverse synthetic datasets, demonstrating their efficiency and effectiveness in maximizing delivery profits.

---

## Installation

Clone the repository and follow these steps to build the project:

```bash
git clone https://github.com/bettisfr/flysidekickproblem_generalized
cd flysidekickproblem_generalized
cmake -B build
cmake --build build
./build/dcoss
```

---

## Citation

If you use this repository, please cite our paper:

```bibtex
@inproceedings{betti2025DCOSS,
  title={Single- and Multi-Depot Optimization for UAV-Based IoT Data Collection in Neighborhoods},
  author={Betti Sorbelli, Francesco and Ghobadi, Sajjad and Palazzetti, Lorenzo and Pinotti, Cristina M.},
  booktitle={21st IEEE International Conference on Distributed Computing in Smart Systems and the Internet of Things (DCOSS-IoT 2025)},
  pages={1-8},
  year={2025},
  organization={IEEE},
  note={Under Review}
}
```

---

## Contact Us

For questions, feedback, or collaboration opportunities, feel free to reach out to us:

- **Francesco Betti Sorbelli**: [francesco.bettisorbelli@unipg.it](mailto:francesco.bettisorbelli@unipg.it)
- **Sajjad Ghobadi**: [sajjad.ghobadibabi@unipg.it](mailto:sajjad.ghobadibabi@unipg.it)
- **Lorenzo Palazzetti**: [lorenzo.palazzetti@unipg.it](mailto:lorenzo.palazzetti@unipg.it)
- **Cristina M. Pinotti**: [cristina.pinotti@unipg.it](mailto:cristina.pinotti@unipg.it)
