**Usage**
Open a terminal in the x64\Debug directory and run:
".\LBVH_ParallelCollisions.exe <numberOfParticles> <METHOD>" (excluding the quotes)
Example : .\LBVH_ParallelCollisions.exe 5000 CUDA

Arguments :
<numberOfParticles> - Number of particles to simulate (e.g., 5000).
<METHOD> - Parallelization method:
            SERIAL  : Single-threaded execution
            OMP     : Multi-threaded with OpenMP
            CUDA    : Multi-threaded with CUDA

**Requirements**
Visual Studio 2022 or another IDE
CUDA Toolkit
A GPU with CUDA support (NVIDIA graphic cards)

*Note : if there is no LBVH_ParallelCollisions.exe, then build the project in Debug mode
