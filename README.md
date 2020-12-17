# Modeling and Control of Mixed Traffic Flow

In this project, dynamical modeling and fundamental properties are investigated for ring-road mixed traffic systems with Connected and Autonomous Vehicle (CAVs) and human-driven vehicles (HDVs). Optimal control strategies are designed for CAVs with a system-level consideration, i.e., 

1. dissipating traffic waves and 
2. improving the performance of the entire mixed traffic flow. 

The Modeling package, [YALMIP](https://yalmip.github.io/), and the optimization solver, [MOSEK](https://www.mosek.com/), are needed to solve the semidefinite program in controller synthesis

* [Topic 1: Smoothing Traffic Flow via Autonomous Vehicles](#Topic-1)
* [Topic 2: Structured Optimal Control of Autonomous Vehicles](#Topic-2)
* [Topic 3: Cooperative Formation of Multiple Autonomous Vehicles: beyong platooning](#Topic-3)


## Topic 1: Smoothing Traffic Flow via Autonomous Vehicles <a name="Topic-1"></a>

Although large-scale numerical simulations and small-scale experiments have shown promising results, a comprehensive theoretical understanding to smooth traffic flow via AVs is lacking. In this paper, from a control-theoretic perspective, we establish analytical results on the **controllability**, **stabilizability**, and **reachability** of a mixed traffic system consisting of HDVs and AVs in a ring road.


Schematic for the mixed traffic system is as follows.

<img src="docs/img/system_schematic_1AV.PNG" align="center" width="60%"/>

### reference
- Zheng, Y., Wang, J., & Li, K. (2020). Smoothing traffic flow via control of autonomous vehicles. *IEEE Internet of Things Journal*, *7*(5), 3882-3896.[[pdf](https://wangjw18.github.io/files/2018-arXiv.pdf)]

Two demonstrations are shown below:

<img src="docs/img/all_HDVs.gif" align="center" width="80%"/>

<p align="center">All the vehicles are HDVs: traffic wave emerges      </p>

<img src="docs/img/one_AV.gif" align="center" width="80%"/>

<p align="center">There is one AV: dissipating traffic wave         </p>

## Toptic 2: Structured Optimal Control of Autonomous Vehicles <a name="Topic-2"></a>

Due to the limit of communication abilities in practice, the CAV can only receive partial information of the
global traffic system for its feedback. Therefore, it is important to consider the **local available information** of the neighboring vehicles. Utilizing limited information exchange to control a large-scale network system leads to the notion of **structured controller design**.

Illustration for structured constraints under limited communication abilities is as follows.

<img src="docs/img/structured_control.PNG" align="center" width="60%"/>

### reference
- Wang, J., Zheng, Y., Xu, Q., Wang, J., & Li, K. (2020). Controllability Analysis and Optimal Control of Mixed Traffic Flow with Human-driven and Autonomous Vehicles. *IEEE Transactions on Intelligent Transportation Systems, 1-15*.[[pdf](https://wangjw18.github.io/files/2020-TITS.pdf)]
- Wang, J., Zheng, Y., Xu, Q., Wang, J., & Li, K. (2019, June). Controllability analysis and optimal controller synthesis of mixed traffic systems. In *2019 IEEE Intelligent Vehicles Symposium (IV)* (pp. 1041-1047). IEEE. [[pdf](https://wangjw18.github.io/files/2019-IV.pdf)] [[poster](https://wangjw18.github.io/files/2019-IV-poster.pdf)]


## Topic 3: Cooperative Formation of Multiple Autonomous Vehicles: beyong platooning <a name="Topic-3"></a>

In mixed traffic flow, the prevailing platooning of multiple AVs is not the only choice for cooperative formation. We re-design the control strategies of AVs in different formations and investigate the optimal formation of multiple AVs using set-function optimization. Two predominant optimal formations, i.e., **uniform distribution** and **platoon formation**, emerges from extensive numerical experiments.

<img src="docs/img/uniform_distribution.gif" align="center" width="80%"/>

<p align="center">Uniform Distribution</p>

<img src="docs/img/platoon_formation.gif" align="center" width="80%"/>

<p align="center">Platoon Formation</p>

### references
- Li, K., Wang, J., & Zheng, Y. (2020). Cooperative Formation of Autonomous Vehicles in Mixed Traffic Flow: Beyond Platooning. *arXiv preprint arXiv:2009.04254*.[[pdf](https://wangjw18.github.io/files/2020-arXiv.pdf)]
- Li, K., Wang, J., & Zheng, Y. (2020). Optimal Formation of Autonomous Vehicles in Mixed Traffic Flow. *In 21st IFAC World Congress*. [[pdf](https://wangjw18.github.io/files/2020-IFAC.pdf)] [[slides](https://wangjw18.github.io/files/2020-IFAC-slides.pdf)]

## Contacts

Related project: [LCC](https://github.com/wangjw18/LCC) (Leading Cruise Control).





