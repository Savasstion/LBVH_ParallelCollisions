# LBVH Parallel Collisions

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

---

## Notes

- If `LBVH_ParallelCollisions.exe` is missing, build the project in **Debug** mode.  
