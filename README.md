# Extended Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

This project works with Udacity self driving simulator (for term 2) and
uses extended Kalman filter in order to predict vehicle position from
radar and lidar sensors.

## The algorithm
The core algorithm this project uses the initial measurements in order to initialize the Kalman filter.
The main idea is to predict first and then update the location and state using the current measurements.

Because the measurements can be sent from two different sensors that work in different coordinate systems,
the update function for each sensor need to differ:
* lidar measurements update method is using regular Kalman filter because this sensor has a linear state function h(x).
* radar measurements update method is using extended Kalman filter because this sensor has a non-linear state function h(x) (arctg function)

After the measurements update the program calculates the RMSE in order to analyze the accuracy of the algorithm.


