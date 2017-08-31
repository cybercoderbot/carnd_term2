# PID Controller

## Introduction

In this project, a PID controller is built to control the steering and throttle of a car in a driving simulator, to enable the car to drive safely and smoothly on the track. To achieve this, a robust control system should be implemented with hyper-parameters properly tuned. 

## PID control
A PID (proportional–integral–derivative) controller is a control system that continuously calculates an error and applies a correction based on proportional (P), integral (I), and derivative (D) terms. In this project, Cross-Track Error (CTE), which denotes the car's offset from the lane center, is used for error measurement.

* The P (proportional) component correct the car position by subtracting a value that is proportional to the CTE. It effects the steering of the car most directly. For example, if the car is supposed to drive forward but steers to the left, by substracting the steering error, the car will correct by a small amount to the right, and vise versa. 

* If the system only has the P component, the car is easily get over corrected, thus nevigate to the opposite direction and eventually drives in a zig-zag pattern. The D (differential) component helps to prevent the car from overshooting the lane center and enable a smoother steering behavior.

* The I (integral) component alleviates error caused by steering drift. This bias error prevents the PD controller from reaching the center of the lane. The I conponent compensate for this type of cumulative error and present cumulative value of the error. 

## Parameter Tuning
Hyperparameters were tuned manually at the beginning to enable to the car drives at least on the track. Once the steering is safe, an aotumatic tuning method, twiddle, is implemented to help the car drives more reliably. Twiddle is an algorithm that tries to find a good choice of parameters for an algorithm that returns an error. It's simple to implement and  gradient calculation is not required.


## Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 


