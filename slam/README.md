# Package: slam

Authors: Aakin Desai and Nithin Gunamgari

## Package Summary

This incorporates different slam techniques on turtlebot3.

* EKF slam from scratch

* RTABMAP with 2D LiDAR scanner

* Frontier Exploration with 2D LiDAR scanner.

### EKF SLAM

For EKF SLAM, refer to [this SLAM resource](https://ieeexplore.ieee.org/document/938381) and [this EKF resource](https://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf) for the details on notation.

**The process model:**

In the process of observing a landmark, the following kinematics is used to predict the vehicle state.

<img src="https://render.githubusercontent.com/render/math?math=\dot{x} = Vcos(\phi), \dot{y} = Vsin(\phi), \dot{\phi} = \frac{Vtan(\gamma)}{L}">

<img src="https://render.githubusercontent.com/render/math?math=\left[\begin{array}{c} x(k%2B1) \\ y(k%2B1) \\ \phi(k%2B1) \end{array} \right]=">  <img src="https://render.githubusercontent.com/render/math?math=\left[\begin{array}{c} x(k)%2B \Delta TV(k)cos(\phi) \\ y(k)%2B \Delta TV(k)sin(\phi) \\ \phi(k)%2B \frac{\Delta TV(k)tan(\gamma)}{L} \end{array} \right]">

The landmarks in the environment are assumed to be stationary point targets. The landmark process model is
thus:

<img src="https://render.githubusercontent.com/render/math?math=\left[\begin{array}{c} x_i(k%2B1) \\ y_i(k%2B1)\end{array} \right]="> <img src="https://render.githubusercontent.com/render/math?math=\left[\begin{array}{c} x_i(k) \\ y_i(k)\end{array} \right]">

**The observation model:**

It returns the range and bearing to a landmark i. The last terms are the noise incorporated. (Here, we used fake sensing data with added noise). 

<img src="https://render.githubusercontent.com/render/math?math=r_i(k) = \sqrt{(x_i - x_r(k))^2 %2B (y_i - y_r(k))^2} %2B w_r(k)">

<img src="https://render.githubusercontent.com/render/math?math=\theta_i(k) = arctan(\frac{y_i - y_r(k)}{x_i - x_r(k)}) - \phi(k) %2B w_\theta(k)">

**Time update:**

Here we project the state and covariance estimates from the previous time step k-1 to the current time step k. The non-linear function relates the states at k-1 and k time steps.

<img src="https://render.githubusercontent.com/render/math?math=\hat{x_k^-} = f(\hat{x_{k-1}^-},u_k-1,0)">

Project the error covariance ahead

<img src="https://render.githubusercontent.com/render/math?math=P_k^- = A_k P_{k-1} A_k^T %2B W_k Q_{k-1} W_k^T">

**Measurement update:**

To correct the state and covariance estimates with the measurement

Compute the Kalman gain
<img src="https://render.githubusercontent.com/render/math?math=K_k = P_k^- H_k^T (H_k P_k^- H_k^T %2B V_k R_k V_k^T)^{-1}">

Update estimate with measurement zk
<img src="https://render.githubusercontent.com/render/math?math=\hat{x_k} = \hat{x_h^-} %2B K_k(z_k - h(\hat{x_k^-},0))">

Update the error covariance
<img src="https://render.githubusercontent.com/render/math?math=P_k = (1 - K_k H_k)P_k^-">

Loop over the above updates for every time step.

EKF Slam with Gazebo landmarks
![gzbslam](./media/path.png =400x400)

<img src="media/path.png" width="200">

Green block - Odometry with black path

Red block - EKF Slam with green path

Robot - Turtlebot3 gazebo path in red

To run this, clone this repository (make sure you have all the turtlebot dependecies installed). 

Launch the following files in order

- `roslaunch turtlebot3_gazebo turtlebot3_world.launch`

- `roslaunch slam slam.launch`

### RTABMAP 

RTAB-Map is a RGB-D SLAM approach with real-time constraints.

RTAB-Map (Real-Time Appearance-Based Mapping) is a RGB-D Graph SLAM approach based on a global Bayesian loop closure detector.

However, we did not have an RGB-D camera to implement RTAB-Map completely. We simplified the system to a 2D-LiDAR by turning off nodes related to loop closure.

To use RTAB-Map, go to this [resource](http://wiki.ros.org/rtabmap) to install the package and [here](http://wiki.ros.org/rtabmap_ros) to know how to use it.

To run this, clone this repository (make sure you have all the turtlebot dependecies installed). 

Launch the following files in order

- `roslaunch turtlebot3_gazebo turtlebot3_world.launch`

- `roslaunch slam slam.launch`
