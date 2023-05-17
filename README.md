# DiffTune: Auto-Tuning through Auto-Differentiation

This is a DiffTune toolset for controller auto-tuning using sensitivity propagation. This toolset is intended to facilitate users' DiffTune applications in two ways. First, it enables the automatic generation of the partial derivatives required for sensitivity propagation. In this way, a user only needs to specify the dynamics and controller, eliminating the need to manually provide partial derivatives. Second, we provide examples that demonstrate the applications of DiffTune and a template to ease the quick deployment to other applications. 

Details of the DiffTune can be found in:<br />
[DiffTune: Auto-Tuning through Auto-Differentiation](https://arxiv.org/abs/2209.10021)<br />
[DiffTune+: Hyperparameter-Free Auto-Tuning using Auto-Differentiation](https://arxiv.org/abs/2212.03194)<br />

If you think this toolset is helpful to your research/application, please cite:<br />
```
@article{cheng2022difftune,
  title={DiffTune: Auto-Tuning through Auto-Differentiation},
  author={Cheng, Sheng and Kim, Minkyung and Song, Lin and Wu, Zhuohuan and Wang, Shenlong and Hovakimyan, Naira},
  journal={arXiv preprint arXiv:2209.10021},
  year={2022}
}
@article{cheng2023difftunePlus,
  title={DiffTune+: Hyperparameter-Free Auto-Tuning using Auto-Differentiation},
  author={Cheng, Sheng and Song, Lin and Kim, Minkyung and Wang, Shenlong and Hovakimyan, Naira},
  journal={accepted by 5th Annual Learning for Dynamics and Control Conference},
  year={2023}
}
```

## Prerequisites

You need to install [CasAdi](https://web.casadi.org/get/) on you computer (Make sure you add CasADi's directory to your MATLAB's path by
```addpath('<yourpath>/<casadi-folder-name>');savepath;```). We will use the autogenerated C code by CasAdi and compile it into mex in MATLAB. Make sure to configure your MATLAB's C compiler by
```mex -setup c```.

## Run examples

We offer two examples: a quadrotor model and a Dubin's car model. Simply navigate to one of the folders under ```/examples```. Take the quadrotor case as an illustrative example, first run ```QuadrotorAutoGeneration.m```. This script automatically generates the C code for evaluating the Jacobians in sensitivity propagation and compile it into mex files, which are available under the subfolder ```/mex``` once the script finishes. Now you can run ```runDiffTune.m``` under ```/examples/quadrotor```. You should be able to see the loss is printed out at each iteration while a figure updates the tracking performance and RMSE of the controller. You can turn on the video generation option that will record the tracking performance and RMSE reduction at run time. We use the [geometric controller](https://ieeexplore.ieee.org/document/5717652) by Taeyoung Lee et al and modified the [source code](https://github.com/fdcl-gwu/decoupled-yaw-controller-comparison/tree/master/FDCL) to our use case.

## Use the template
A template for the usage of DiffTune on custom systems and controllers are provided in ```/template```. Users are recommended to fill in the ```dynamics.m``` and ```controllers.m``` first and run the ```runDiffTune.m``` (with the DiffTune related components commented out) to make sure the basic simulation can run as expected. The next step is to fill in the ```templateAutoGeneration.m``` with discretized dynamics and run this script to generate and compile the functions for online Jacobians evaluation. Once done, you should see ```*.mex``` and ```*.c``` files under the directory ```/template/mex```. Now, you can retain the commented out sections in ```runDiffTune.m``` and run this script for your own DiffTune application.

## Issues/Questions/Suggestions
Feel free to open up an issue if you run into troubles. 

# Authors

**[Sheng Cheng](https://github.com/Sheng-Cheng)**
**[Lin Song](https://www.linkedin.com/in/lin-song96/)**
**[Minkyung Kim](https://www.linkedin.com/in/kmk7733/)**

## License

This project is licensed under the GPL-3.0 License - see the [LICENSE](LICENSE) file for details

[![Hits](https://hits.seeyoufarm.com/api/count/incr/badge.svg?url=https%3A%2F%2Fgithub.com%2FSheng-Cheng%2FDiffTuneOpenSource&count_bg=%2379C83D&title_bg=%23555555&icon=&icon_color=%23E7E7E7&title=hits&edge_flat=false)](https://hits.seeyoufarm.com)
