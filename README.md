**Structure of this repository code:** <br>
1. Ipopt_home 
    * Information on how to install IPOPT and IPOPT wrapper for python. IPOPT is the optimization solver currently in use.
2. RTD_cartographer_ws
    * ROS workspace launch files for the SLAM software Google Cartographer, specific to the ROAHM lab rover.
3. RTD_ws
    * RTD (Reachability Based Trajectory Design) ROS workspace.
    * `FRS_intersect_ipopt.py` implements the online RTD pipeline.
4. matlab_sim
    * offline matlab code specific to the ROAHM lab rover used to generate and validate the FRS (Forward Reachable Set) loaded at runtime. This code should be run in the code base at https://github.com/skousik/RTD_tutorial.

**For user information specific to the ROAHM lab rovers, including how to run RTD:** <br>
1. https://docs.google.com/document/d/1erhFfmkBvs7w6wD4rrPJ-PZUyJ2Op4C7-7hIVi4XcL0/edit?usp=sharing
