# Radar Tracking

[comment]: *******************************************************************

## Problem Statement

This is an example of the extended Kalman Filter. 
We want to find the position and altitude of an object from it's measured slant range. 
The measurement model for this object is non-linear. 

For simplicity, the object is assumed to be moving in 3D with constant speed and altitude.
However, the velocity and altitude are unknown. 

                           Object  ==> direction of velocity
                             /|
                            / |
                           /  |
                          /   |
                         /    |
                        /     |
                       /      |
                      /       | h_alt
                     /        |
                    /         |
                   /          |
                  /           |
                 /   x_trk    |
                /-------------|
              radar

[comment]: *******************************************************************

## System model

### System state variables
```
    [horizontal distance]   [x1]
x = [velocity           ] = [x2]
    [altitude           ]   [x3]
```

### State transition model

Since velocity and altitude are constant, the equation of motion becomes: 
```
                    [rate_of_change(x1)]
rate_of_change(x) = [rate_of_change(x2)]
                    [rate_of_change(x3)]
                    
                    [rate_of_change(horizontal_distance)]
                  = [rate_of_change(velocity)           ]
                    [rate_of_change(altitude)           ]  
                    
                    [0  1  0]   [x1]   [0 ]
                  = [0  0  0] * [x2] + [w1]
                    [0  0  0]   [x3]   [w2]
                    
                    [x2]
                  = [w1] 
                    [w2] 
                    
                    [velocity]
                  = [noise of rate_of_change(velocity)]
                    [noise of rate_of_change(altitude)]  
```

We derive the system in this way:
* `rate_of_change(horizontal_distance) = rate_of_change(x1) = velocity`
* Velocity is assumed to be constant, meaning that in an ideal system,
  `rate_of_change(velocity) = 0`. Bit in the real world, `rate_of_change(velocity)`
  has some slight variations due to various sources of error. This is modeled
  as system noise: `rate_of_change(velocity) = rate_of_change(x2) = w1`.
* Altitude is also assumed to be constant, meaning that in an ideal system,
  `rate_of_change(altitude) = 0`. Bit in the real world, `rate_of_change(altitude)`
  has some slight variations due to various sources of error. This is modeled
  as system noise `w2`: `rate_of_change(altitude) = rate_of_change(x3) = w2`.
  
### State-to-measurement model

The physical quantity measured by the radar is slant range, defined as:
```
r = slant_range
  = sqrt(horizontal_distance^2 + altitude^2) + measurement_noise_radar
  = sqrt(x1*x1 + x3*x3) + v
  = h(x) + v
```
where `h(x) = sqrt(x1*x1 + x3*x3)`.

The system model is not linear, so we cannot apply the linear Kalman filter to this problem.

[comment]: *******************************************************************

## Extended Kalman filter function

Since the system model is linear, a linear Kalman filter is used for the prediction
of the state variable in this extended Kalman filter algorithm.

```
\hat{x_predicted}_{k+1} = f(\hat{x}_k)
```
which is equivalent to
```
\hat{x_predicted}_{k+1} = A * \hat{x}_k
```
The matrix `A` is used in place of the Jacobian of the function `f(x)`.

The extended Kalman filter is an algorithm for a discrete system:
```
x_{k+1} = f(x_k) + w_k
z_k     = h(x_k) + v_k
```
and the following functions are equivalent in the linear Kalman filter:
```
A * x_k is equivalent to f(x_k)
H * x_k is equivalent to h(x_k)
```

### System model

The system model linear but not discrete, so we discretized it with Euler integration within the interval `dt`.
Numerical integration could be used if a more accurate result is needed.
```
              [0  1  0]
A = I_{3x3} + [0  0  0] * dt
              [0  0  0]
```

### Measurement model

The measurement model is non-linear, so matrix `H` is obtained by calculating the Jacobian of 
the state-to-measurement equation:

```
H = [dh/dx1  dh/dx2  dh/dx3]

    [       x1                   x2        ]
  = [----------------  0  ---------------- ]
    [sqrt(x1^2 + x3^2)    sqrt(x1^2 + x3^2)]
```
