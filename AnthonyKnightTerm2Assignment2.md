## **Unscented Kalman Filter Project**



### Writeup by Tony Knight - 2017/08/14


The [UKF project code](https://github.com/teeekay/CarND-Unscented-Kalman-Filter-Project/tree/master/src) was mainly developed using code taken from the online classes.

The final RMSE values I obtained in the simulator data were:

|Dataset|rmse_x|rmse_y|rmse_vx|rmse_vy|
|-|-|-|-|-|
|1|0.0688|	0.0812|	0.2793|	0.2305|
|2|	0.0709|	0.0638|	0.4909|	0.2633|

These values were produced when the following process noise settings were used:

|std_a_ | std_yawdd_|
|---|--|
| 1.0 | PI / 4 |

and the initial covariance matrix P_ was set to:

|col 1|col 2|col 3|col 4|col 5|
|--|--|--|--|--|
|0.02| 0  | 0 | 0 | 0|
|0 |0.02| 0  | 0 | 0 |
|0|0|1|0|0|
|0|0|0|1|0|
|0|0|0|0|1|


The calculated NIS results produced with these settings appeared to match relatively well with the expected 95% chi-square values for Laser (5.99) and Radar (7.81) as seen in the graphs below.

---

<img src="https://github.com/teeekay/CarND-Unscented-Kalman-Filter-Project/blob/master/Output/NIS_Radar.png?raw=true"  width=700>

<i><u>Figure 1: Graphed Radar NIS values retrieved when running Dataset 1 on Simulator</u></i>

<img src="https://github.com/teeekay/CarND-Unscented-Kalman-Filter-Project/blob/master/Output/NIS_Laser.png?raw=true"  width=700>

<i><u>Figure 2: Graphed Laser NIS values retrieved when running Dataset 1 on Simulator</u></i>


---



The full output values for a run using the UKF code can be found in the [csv file](https://github.com/teeekay/CarND-Unscented-Kalman-Filter-Project/blob/master/Output/test20170814r1.csv)


Also - please look at my use of the UKF code to [catch the runaway car](https://github.com/teeekay/CarND-Catch-Run-Away-Car-UKF/blob/master/AnthonyKnight_UKF_Project_RunawayCar.md)
