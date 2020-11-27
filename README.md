# MCMT
In this project, dynamical modeling and fundamental properties are investigated for ring-road mixed traffic systems with Connected and Autonomous Vehicle (CAVs) and human-driven vehicles (HDVs). Optimal control strategies are designed for CAVs with a system-level consideration, i.e., dissipating traffic waves and improving the performance of the entire mixed traffic flow. 

An existing solver, [MOSEK](https://www.mosek.com/), is needed to be installed first to numerically solve the optimization problem and obtain the optimal controller.

## Smoothing Traffic Flow via AV

Although large-scale numerical simulations and small-scale experiments have shown promising results, a comprehensive theoretical understanding to smooth traffic flow via AVs is lacking. In this paper, from a control-theoretic perspective, we establish analytical results on the **controllability**, **stabilizability**, and **reachability** of a mixed traffic system consisting of HDVs and AVs in a ring road.

<img src="docs/img/system_schematic_1AV.PNG" align="center" width="70%"/>

<center>System schematic for mixed traffic system with one AV</center>
<img src="docs/img/all_HDVs.gif" align="center" width="80%"/>

<center>All the vehicles are HDVs</center>
<img src="docs/img/one_AV.gif" align="center" width="80%"/>

<center>There is one AV</center>
## Structured Optimal Control of AV

Due to the limit of communication abilities in practice, the CAV can only receive partial information of the
global traffic system for its feedback. Therefore, it is important to consider the **local available information** of the neighboring vehicles. Utilizing limited information exchange to control a large-scale network system leads to the notion of **structured controller design**.

<img src="docs/img/structured_control.PNG" align="center" width="70%"/>

<center>Structured constraints under limited communication abilities</center>
## Cooperative Formation of Multiple AVs

In mixed traffic flow, the prevailing platooning of multiple AVs is not the only choice for cooperative formation. We re-design the control strategies of AVs in different formations and investigate the optimal formation of multiple AVs using set-function optimization. Two predominant optimal formations, i.e., **uniform distribution** and **platoon formation**, emerges from extensive numerical experiments.

<img src="docs/img/uniform_distribution.gif" align="center" width="80%"/>

<center>Uniform Distribution</center>
<img src="docs/img/platoon_formation.gif" align="center" width="80%"/>

<p align="center">Platoon Formation</p>

## Publications

**Journal papers**

- Zheng, Y., Wang, J., & Li, K. (2020). Smoothing traffic flow via control of autonomous vehicles. *IEEE Internet of Things Journal*, *7*(5), 3882-3896.[[pdf](https://wangjw18.github.io/files/2018-arXiv.pdf)]
- Wang, J., Zheng, Y., Xu, Q., Wang, J., & Li, K. (2020). Controllability Analysis and Optimal Control of Mixed Traffic Flow with Human-driven and Autonomous Vehicles. *IEEE Transactions on Intelligent Transportation Systems, 1-15*.[[pdf](https://wangjw18.github.io/files/2020-TITS.pdf)]
- Li, K., Wang, J., & Zheng, Y. (2020). Cooperative Formation of Autonomous Vehicles in Mixed Traffic Flow: Beyond Platooning. *arXiv preprint arXiv:2009.04254*.[[pdf](https://wangjw18.github.io/files/2020-arXiv.pdf)]

**Conference papers**

- Li, K., Wang, J., & Zheng, Y. (2020). Optimal Formation of Autonomous Vehicles in Mixed Traffic Flow. *In 21st IFAC World Congress*. [[pdf](https://wangjw18.github.io/files/2020-IFAC.pdf)] [[slides](https://wangjw18.github.io/files/2020-IFAC-slides.pdf)]
- Wang, J., Zheng, Y., Xu, Q., Wang, J., & Li, K. (2019, June). Controllability analysis and optimal controller synthesis of mixed traffic systems. In *2019 IEEE Intelligent Vehicles Symposium (IV)* (pp. 1041-1047). IEEE. [[pdf](https://wangjw18.github.io/files/2019-IV.pdf)] [[poster](https://wangjw18.github.io/files/2019-IV-poster.pdf)]

## Contacts

Related project: [LCC](https://github.com/wangjw18/LCC) (Leading Cruise Control).





