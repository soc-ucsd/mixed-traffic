Here, we list the results of experimentation with the Python simulation. 

# Penetration Rate
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

## Raw Data

### Platoon Case

#### Linear Optimal Control

<table>
<tr><td> 
 
|      20 vehicles     |                   |                     |                    |
|:--------------------:|-------------------|---------------------|--------------------|
| Penetration Rate (%) | Settling Time (s) | Maximum Spacing (m) | Control Energy (J) |
|           0          |        100        |          -          |          -         |
|           5          |       63.95       |        39.99        |        509.5       |
|          10          |        60.3       |        28.23        |       393.36       |
|          15          |       51.13       |        24.93        |       263.53       |
|          20          |       48.97       |        24.96        |       236.01       |   
|          25          |       48.23       |        25.18        |        232.4       |   
|          30          |       51.55       |        25.31        |        230.9       |   
|          35          |       54.23       |        25.37        |       230.17       |   
|          40          |       56.26       |        25.43        |       229.89       |   
|          45          |       58.09       |        25.49        |       229.99       |  
|          50          |       59.36       |        25.58        |       230.11       |   
|          75          |       59.35       |        25.56        |       226.92       |  
|          100         |       52.36       |        32.33        |       205.54       |   

</td><td> 

|      45 vehicles     |                   |                     |                    |   
|:--------------------:|-------------------|---------------------|--------------------|
| Penetration Rate (%) | Settling Time (s) | Maximum Spacing (m) | Control Energy (J) |   
|           0          |        100        |          -          |          -         |   
|           5          |       76.99       |        24.32        |        288.5       |   
|          10          |       75.91       |        24.94        |       233.82       |   
|          15          |       73.43       |        25.28        |       230.63       |   
|          20          |       62.86       |        25.43        |       230.36       |   
|          25          |       66.04       |        25.52        |       230.13       |   
|          30          |       69.37       |        25.55        |       229.93       |   
|          35          |       71.98       |        25.57        |       230.03       |   
|          40          |       74.05       |        25.58        |       230.11       |   
|          45          |       76.17       |        25.57        |        230.2       |   
|          50          |       76.99       |        25.56        |       230.24       |   
|          75          |       77.03       |        25.49        |       231.38       |   
|          100         |        65.7       |        32.35        |       206.49       |   

</td><td> 

|      70 vehicles     |                   |                     |                    |  
|:--------------------:|-------------------|---------------------|--------------------|
| Penetration Rate (%) | Settling Time (s) | Maximum Spacing (m) | Control Energy (J) |   
|           0          |        200        |          -          |          -         |   
|           5          |       105.3       |        24.47        |       239.71       |  
|          10          |        102        |        25.26        |       231.05       |   
|          15          |       90.07       |        25.49        |       230.11       |   
|          20          |         78        |        25.55        |       229.99       |   
|          25          |       76.47       |        25.59        |       230.19       |   
|          30          |       79.51       |         25.6        |        230.3       |   
|          35          |        82.7       |        25.61        |       230.41       |   
|          40          |       84.31       |        25.62        |       230.47       |   
|          45          |        85.6       |        25.62        |       230.51       |  
|          50          |       86.16       |        25.61        |       230.54       |  
|          75          |       84.24       |        25.44        |       230.68       |   
|          100         |       71.66       |        32.35        |       206.59       |   

</td></tr> 
</table>
 
#### Follow-stopper

<table>
<tr><td> 

|      20 vehicles     |                   |                     |                    |  
|:--------------------:|-------------------|---------------------|--------------------|
| Penetration Rate (%) | Settling Time (s) | Maximum Spacing (m) | Control Energy (J) |   
|           0          |        100        |          -          |          -         |   
|           5          |       73.82       |        44.12        |        44.74       |  
|          10          |       58.41       |        45.07        |        66.46       |  
|          15          |       58.08       |        46.61        |        71.42       | 
|          20          |       50.76       |        48.82        |        71.61       |  
|          25          |       36.99       |        49.07        |        71.61       |  
|          30          |       36.99       |        49.07        |        71.61       |  
|          35          |       36.99       |        49.07        |        71.61       | 
|          40          |       36.99       |        49.07        |        71.61       | 
|          45          |       36.99       |        49.07        |        71.61       | 
|          50          |       36.99       |        49.07        |        71.61       | 
|          75          |       36.99       |        49.07        |        71.61       | 
|          100         |        30.8       |        46.43        |       143.57       | 

 </td><td>

|      45 vehicles     |                   |                     |                    |   
|:--------------------:|-------------------|---------------------|--------------------|
| Penetration Rate (%) | Settling Time (s) | Maximum Spacing (m) | Control Energy (J) |   
|           0          |        100        |          -          |          -         |   
|           5          |       89.49       |        49.07        |        71.42       |  
|          10          |       36.99       |        49.07        |        71.61       |  
|          15          |       36.99       |        49.07        |        71.61       | 
|          20          |       36.99       |        49.07        |        71.61       | 
|          25          |       36.99       |        49.07        |        71.61       |  
|          30          |       36.99       |        49.07        |        71.61       |  
|          35          |       36.99       |        49.07        |        71.61       | 
|          40          |       36.99       |        49.07        |        71.61       |  
|          45          |       36.99       |        49.07        |        71.61       |  
|          50          |       36.99       |        49.07        |        71.61       |  
|          75          |       36.99       |        49.07        |        71.61       |  
|          100         |       29.45       |        46.43        |       143.57       |   

 </td><td>

|      70 vehicles     |                   |                     |                    |   
|:--------------------:|-------------------|---------------------|--------------------|
| Penetration Rate (%) | Settling Time (s) | Maximum Spacing (m) | Control Energy (J) |   
|           0          |        200        |          -          |          -         |   
|           5          |       112.46      |        49.07        |        71.61       |   
|          10          |       36.99       |        49.07        |        71.61       |   
|          15          |       36.99       |        49.07        |        71.61       |   
|          20          |       36.99       |        49.07        |        71.61       |   
|          25          |       36.99       |        49.07        |        71.61       |  
|          30          |       36.99       |        49.07        |        71.61       |   
|          35          |       36.99       |        49.07        |        71.61       |   
|          40          |       36.99       |        49.07        |        71.61       |   
|          45          |       36.99       |        49.07        |        71.61       |   
|          50          |       36.99       |        49.07        |        71.61       |   
|          75          |       36.99       |        49.07        |        71.61       |   
|          100         |       28.62       |        46.43        |       143.57       |

 </td></tr>
 </table>
 
| Method      | Description                          |
| ----------- | ------------------------------------ |
| `GET`       | :material-check:     Fetch resource  |
| `PUT`       | :material-check-all: Update resource |
| `DELETE`    | :material-close:     Delete resource |

