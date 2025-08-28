# LBVH and Parallel Collisions

## Overview

This project implements an Linear Bounding Volume Hierarchy (LBVH) by Karras (2012) to construct a Bounding Volume Hierarchy (BVH) in parallel to accelerate collision detection in a scene as well as doing collision resolutions in parallel as well. With all these, a scene can process collisions of thousands of particles on the scene optimally.

<img width="784" height="791" alt="ParallelCollisionsDemo" src="https://github.com/user-attachments/assets/b0b117d8-e2df-48bd-98e4-15fd56cc70b8" />

(Image of collisions being simulated for 8000 particles using CUDA with a RTX 4070)


## Usage

Open a terminal in the `x64\Debug` directory and run:

```bash
.\LBVH_ParallelCollisions.exe <numberOfParticles> <METHOD>
```

Example:

```bash
.\LBVH_ParallelCollisions.exe 5000 CUDA
```

---

## Arguments

- **`<numberOfParticles>`** — Number of particles to simulate (e.g., `5000`).  
- **`<METHOD>`** — Parallelization method:  
  - `SERIAL` : Single-threaded execution  
  - `OMP` : Multi-threaded with OpenMP  
  - `CUDA` : Multi-threaded with CUDA  

---

## Requirements

- **Visual Studio 2022** (or another compatible IDE)  
- **CUDA Toolkit**  
- **NVIDIA GPU** with CUDA support
- **Windows OS**

---

## Notes

- If `LBVH_ParallelCollisions.exe` is missing, build the project in **Debug** mode.

## References
Karras, T. (2012). Maximizing parallelism in the construction of BVHs, octrees, and k-d trees. High Performance Graphics, 33–37. https://doi.org/10.5555/2383795.2383801

