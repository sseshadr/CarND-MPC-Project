# CarND-MPC-Project
Use MPC control to drive a simulated car on its track. Perform the constrained optimization problem in the loop to compute actuator values.

#### NOTE: This project involves the Term 2 Simulator which can be downloaded [here] (https://github.com/udacity/self-driving-car-sim/releases). It also depends on the C++ eigen library which can be downloaded [here] (https://github.com/eigenteam/eigen-git-mirror).

[//]: # (Image References)

[image1]: ./readme_media/states1.png "Reference States - Kinematics"
[image2]: ./readme_media/states2.png "Reference States - CTE"
[image3]: ./readme_media/states3.png "Reference States - Heading Error"
[image4]: ./readme_media/NdtDiscussion.png "N-dt Table"
[image5]: ./readme_media/Ndt25_001.gif "N = 25, dt = 0.01"
[image6]: ./readme_media/Ndt10_01.gif "N = 10, dt = 0.1"
[image6]: ./readme_media/pidbetter.gif "Final Simulation with manually tuned parameters"

---

#### Project Notes
Designing a MPC controller included the following:

#### Model:
A kinematics vehicle model was used to predict the system behavior. The states of this model consists of the following:
* Position (x, y)
* Heading (psi)
* Velocity (v)

These are calculated using the following equations where a(t) and delta(t) are actuator varibales. L_f is a system variable that is the distance between the center of mass of the vehicle and it's front axle. 

![alt text][image1]

Next, we append the error terms.
* Cross track error (cte)
* Heading error (epsi)

These are calculated using the following equations:

![alt text][image2]
![alt text][image3]

So in total we have 6 states that we track. 

#### Timestep Length and Elapsed Duration (N & dt):
The rule of thumb while choosing N and dt primarily is a tradeoff between computation effort required to solve the optimization problem and the length of the future horizon that we would like to keep track (N) and the resolution of these predictions (dt). With this, we can draw the following table to better understand how they affect each other.

![alt text][image4]

So we can see that to be able to predict well, we would need to invest in computation time. Originally, I started with an N of 25 and dt of 0.01 (1.25s on the horizon), and the computation effort was too much to result in a stable loop as shown below.

![alt text][image5]

I reduced N and increased dt to 10 and 0.1 respectively (1s on the horizon) and was able to get a stable behavior as shown below.

![alt text][image6]

#### Polynomial fitting and MPC preprocessing:
A 3rd order polynomial was fitted on the waypoints provided by the simulator. To do this, the provided polyfit function was used. Before the polynomial fitting, the waypoints were transformed to the vehicle coordinate system so that the initial x, y and psi could be assumed to be 0. Once we get the coefficients, we evaluate the polynomial at the desired x location (0) to compute the cross track error. The same coefficients are used to compute the heading error as well.

#### MPC with latency:
The MPC worked well for the chosen N and dt values. To deal with the latency, I first tried to play with the weight associated with the different cost parameters. Specifically, I penalized subsquent steering angle parameters to be really high. However this still did not result in a stable system. Then I looked how to better set the reference states when solving for the MPC optimization problem. To do this, I used the kinematics equations as shown above. Instead of setting x, y, psi to 0 and v to the current v, we can use the kinematics equations to calculate a 'future' v for delta time set to the latency time. These incremental values make a big difference in how we initialize the states for the MPC problem and take the latency into account by computing a future state as a reference state.
