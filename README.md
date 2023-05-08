# Decoupled-Yaw Controller Comparison

This repository includes the codes used for generating the results in the paper "Geometric Adaptive Controls of a Quadrotor UAV with Decoupled Attitude Dynamics".
For other controllers used at FDCL, or for other progamming languages, please check [uav_geometric_control](https://github.com/fdcl-gwu/uav_geometric_control) repository.

The repository is structures as follows:
- **FDCL**: includes simulations for two controllers
    - proposed geometric adaptive decoupled-yaw controller controller
    - geometric adaptive controller with coupled-yaw dynamics ([link](https://doi.org/10.1115/1.4030419))
- **Kooijman**: simulation of "Trajectory Tracking for Quadrotors with Attitude Control on S^2 x S^1" ([link](https://doi.org/10.23919/ECC.2019.8795755))
- **Brescianini**: simulation of "Tilt-Prioritized Quadrocopter Attitude
  Control" ([link](https://doi.org/10.1109/TCST.2018.2873224))

- **Common**: functions such as desired trajectories and plotting which are common for all above controllers
- **Data**: data files used to generate plots and table in "Geometric Adaptive Controls of a Quadrotor UAV with Decoupled Attitude Dynamics"
