# Motion Illuminations in AirSim

**Baladitya Swaika | Collaborator: Henil Shelat**

## Project Objective & Overview

To realize the next generation of multimedia displays, or FLS (Flying Light Speck) displays, which are capable of rendering real or virtual 3-dimensional objects by occupying physical volumes in space, we need miniature UAVs that are only hundreds of micrometers in size and have one or more light sources which can be lit in different colors and brightness levels. However, such technology is still under open research, and their fabrication is difficult and expensive due to their small size. Therefore, to understand and analyze the different aspects involved in creating an FLS display, one has to resort to simulations in the present technology landscape.

AirSim is a simulation engine built on top of Unreal Engine which has real-time HITL (hardware-in-the-loop) capabilities for high frequency computations, a modular design for ease of development and a wide variety of sensors for gathering data. It has been used in popular drone racing competitions and supports a wide variety of protocols. The objective of the project is to simulate and analyze FLS flight paths for rendering motion illuminations in AirSim. The project will implement the MOTILL algorithm for rendering the RoseClip point cloud sequence, STAG for staggered charging of FLS drones, and a simple garbage collection mechanism that replaces FLS drones that fail. To analyze flight paths, the key would be to investigate metrics like collision frequencies, total distance traveled, illumination latency, and execution time.

Although this project uses the research conducted by Dr. Shahram Ghandeharizadeh as its foundation, it comes with certain challenges. Firstly, the RoseClip point cloud sequence houses 115 frames of point clouds each of which contain around 65k points. It will be extremely challenging to render so many objects in a simulator at high frame rates (10+ point clouds per second) especially on an intermediate machine with AMD Ryzen 9 and Nvidia RTX3060. It will be crucial to try and optimize renderings for higher frame rates to give a more realistic feel to the rendering. Secondly, the physical effects of downwash would require attention by calibrating for spacing between FLS drones. Finally, a conceptual challenge arises with the positioning of dark FLSs. Both of these can impact the user’s perception of the rendering and needs to be examined further as the project is developed.

## Milestones
- [ ] Feasibiility & Setup (10/12/2022)
- [ ] MOTILL & FLS Drone Lighting (11/4/2022)
- [ ] STAG & Failure Handling (11/23/2022)
- [ ] Analysis & Reporting (11/27/2022)