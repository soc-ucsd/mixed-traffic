# SRIP 2022 Project: Python Simulations and Numerical Experiments

- Authors: Anish Kulkarni, Yanzhi Yao
- Advisor: Yang Zheng
- Slides: [Summer_Research_Conference](https://docs.google.com/presentation/d/1TmJLyyWzn3yRkOp7Arv4sMOJvpLOkLZRWA0jbsv1El4/edit?usp=sharing)

Here, we list some python simulations and numerical experiment results, which was led by Anish Kulkarni and Yanzhi Yao in the summer of 2022. 

## Penetration Rate
The first experiment conducted involved varying the penetration rate of AVs in a mixed traffic system of N vehicles. This variation was done in terms of a percentage value (0%, 5%, 10%, 15%, 20%, 25%, 30%, 35%, 40%, 45%, 50%, 75%, 100%) - as in, what percentage of the N vehicles were autonomous. 


## Setup
The simulation was run for T = 100 seconds for N = 20 and N = 45 vehicles ; and for T = 200 seconds for N = 70 vehicles. 

Each vehicle in the simulation was indexed from 0 to N-1. In all simulations, at T = 20 seconds, the vehicle at index '4' was slowed to a minimum speed of 8m/s. After this, the simulation was allowed to return to a stable state. In all simulations (except 100% penetration rate), the vehicle at index '4' was an HDV.

Once the system stabilized (each vehicle returned to within 3% of its original speed), the settling time, maximum spacing in front of any AV, and the control energy of each AV were calculated. These values were then graphed to identify trends in the data. 

## Modified Values
The penetration rate was varied in the system, as well as the configuration of the AVs (platoon formation / uniform distribution). 

## AV Controllers 
Three AV controllers were tested in the simulation :
 -  FollowerStopper
 -  PI with Saturation
 -  Linear Optimal Control
Each controller was tested for all penetration rates in both configurations (platoon and uniform), for all N values. 

## Experiment Result

### Platoon Case

#### Penetration Rate vs Settling Time

![Alt Text](images/penetration_rate_data/Platoon/Platoon_N=20_settlingtime.png)

![Alt Text](images/penetration_rate_data/Platoon/Platoon_N=45_settlingtime.png)

![Alt Text](images/penetration_rate_data/Platoon/Platoon_N=70_settlingtime.png)

#### Penetration Rate vs Maximum Spacing

![Alt Text](images/penetration_rate_data/Platoon/Platoon_N=20_maximum_spacing.png)

![Alt Text](images/penetration_rate_data/Platoon/Platoon_N=45_maximum_spacing.png)

![Alt Text](images/penetration_rate_data/Platoon/Platoon_N=70_maximum_spacing.png)


#### Penetration Rate vs Control Energy

![Alt Text](images/penetration_rate_data/Platoon/Platoon_N=20_control_energy.png)

![Alt Text](images/penetration_rate_data/Platoon/Platoon_N=45_control_energy.png)

![Alt Text](images/penetration_rate_data/Platoon/Platoon_N=70_control_energy.png)


### Uniform Case

#### Penetration Rate vs Settling Time

![Alt Text](images/penetration_rate_data/Uniform/Uniform_N=20_settlingtime.png)

![Alt Text](images/penetration_rate_data/Uniform/Uniform_N=45_settlingtime.png)

![Alt Text](images/penetration_rate_data/Uniform/Uniform_N=70_settlingtime.png)

#### Penetration Rate vs Maximum Spacing

![Alt Text](images/penetration_rate_data/Uniform/Uniform_N=20_maximum_spacing.png)

![Alt Text](images/penetration_rate_data/Uniform/Uniform_N=45_maximum_spacing.png)

![Alt Text](images/penetration_rate_data/Uniform/Uniform_N=70_maximum_spacing.png)


#### Penetration Rate vs Control Energy

![Alt Text](images/penetration_rate_data/Uniform/Uniform_N=20_control_energy.png)

![Alt Text](images/penetration_rate_data/Uniform/Uniform_N=45_control_energy.png)

![Alt Text](images/penetration_rate_data/Uniform/Uniform_N=70_control_energy.png)


## Raw Data

[Experiment Data](https://docs.google.com/spreadsheets/d/1muLcmCCUdFwuMU2z9FdTxCUI8RmbDvqNfVlfMRN3k_Q/edit?usp=sharing)
   


## Result Graphs
Using all of the raw data obtained through the simulation, we were able to create several graphs detailing the results of the experiment. 

* INSERT IMAGES * 

## Analysis
From the data, we can infer several aspects of each controller and their efficacy - 
- FollowerStopper : This controller performed the best with a larger and larger simulation size. We observed lower energy consumption and a smaller settling time on average for FollowerStopper vehicles. However, the detriment came from the excessive space usage that ocurred - in a single lane, such space usage causes few problems, but in a practical situation, it is unfeasible. 
- Linear Optimal : This controller performed well, with a roughly decreasing trend for settling time and energy usage with size. As such, we can expect it to perform better with a larger penetration rate. However, experiments showed that the best settling time usually occurred at around 30% penetration. Additionally, the space usage was the smallest out of any controller, making the controller more practical. 
- PI with Saturation : This controller had the highest average velocity at the end of the trial, but lacked in all measured aspects of the traffic flow. Therefore, it seems impractical to use it in a real traffic situation.
