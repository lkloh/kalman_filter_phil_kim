# Radar Tracking

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

## System model

System state variables are:
```
    [horizontal distance]   [x1]
x = [velocity           ] = [x2]
    [altitude           ]   [x3]
```

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
  
## Measurement model

The physical quantity measured by the radar is slant range, defined as:
```
r = slant_range
  = sqrt(horizontal_distance^2 + altitude^2) + measurement_noise_radar
  = sqrt(x1*x1 + x3*x3) + v
  = h(x) + v
```
where `h(x) = sqrt(x1*x1 + x3*x3)`.

The system model is not linear, so we cannot apply the linear Kalman filter to this problem.



