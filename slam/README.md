# Package: slam

Authors: Aakin Desai and Nithin Gunamgari

## Package Summary

This incorporates EKF SLAM on the Turtlebot3.

### EKF SLAM

For EKF SLAM, refer to [this SLAM resource](https://ieeexplore.ieee.org/document/938381) and [this EKF resource](https://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf) for the details on notation.

The process model:
In the process of observing a landmark, the following kinematics is used to predict the vehicle state.

<img src="https://render.githubusercontent.com/render/math?math=\dot{x} = Vcos(phi)">
