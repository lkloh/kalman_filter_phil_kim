## Attitute reference system with Extended Kalman Filter

## Introduction

We want to measure the horizontal attitude of a helicopter. 
To estimate the horizontal attitude, we combine measurements from a gyroscope and accelerometer
using an EKF as these instruments have complementary strengths and weaknesses. 

The Kalman filter is designed to calibrate the error accumulated in the gyroscope with an
accelerometer. 

## System model

The state variable `x` is the Euler angles:
```
    [phi  ]   [roll ]
x = [theta] = [pitch]
    [psi  ]   [yaw  ]
```
Only the roll (`phi`) and pitch (`theta`) angles are used to describe the tilted angle
of a horizontal plane, but yaw angle (`psi`) is needed as a state variable.

The relationship between Euler angles and angular velocity is well known is kinematics.

```
[rate_of_change(phi)  ]   [1  sin(phi)*tan(theta)  cos(phi)tan(theta)  ][p]
[rate_of_change(theta)] = [0  cos(phi)             -sin(phi)           ][q] + w
[rate_of_change(psi)  ]   [0  sin(phi)/cos(theta)  cos(phi)/cos(theta) ][r]
                        
                          [p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta)] 
                        = [q*cos(phi) - r*sin(phi)                          ] + w
                          [q*sin(phi)/cos(theta) + r*cos(phi)/cos(theta)    ]
                          
                        = f(x) + w
``` 

This relationship is non-linear -- it contains trigonometric functions that are multiplied together 
It needs to be expressed in discrete form for EKF use. 

## Measurement model

The measurement for calibration is obtained from the accelerometer. 
The accelerometer cannot calculate yaw angle, so measurement model is written as:

```
    [1  0  0][phi  ] 
z = [0  1  0][theta] + v
             [psi  ]
             
  = H*x + v           
```
This is a linear expression, so we can use it as-is.

## Extended Kalman filter function

To linearize the system model expression, we take it's Jacobian. 

The system model is given by:
```
       [f1(x)]        
f(x) = [f2(x)]
       [f3(x)]                       
        
       [p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta)] 
     = [q*cos(phi) - r*sin(phi)                          ]
       [q*sin(phi)/cos(theta) + r*cos(phi)/cos(theta)    ]                     
```

and it's Jacobian is defined as:
```
    [d f1   d f1     d f1 ]
    [-----  -------  -----]
    [d phi  d theta  d psi]
    [                     ]
    [d f2   d f2     d f2 ]
J = [-----  -------  -----]
    [d phi  d theta  d psi]
    [                     ]
    [d f3   d f3     d f3 ]
    [-----  -------  -----]
    [d phi  d theta  d psi]
```
To discretize the Jacobian for use in EKF, we use Euler integration within the interval `dt`.
A more accurate approach would use numerical integration.
```
    [1  0  0]
A = [0  1  0] + J * dt
    [0  0  1] 
```

In general, getting the derivative of a system model to compute the Jacobian is straightforward.
For a simpler system, the derivatives can be obtained analytically.
For a more complex system, numerical methods are needed to obtain the Jacobian,
which could result in higher chances of errors.

## Conclusions


