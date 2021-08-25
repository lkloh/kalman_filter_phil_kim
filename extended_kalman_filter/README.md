# Extended Kalman Filter (EKF)

## Introduction

50 years after the Kalman Filter's development, most new papers published about it fall into these categories:
* New applications for it - most of the papers
* Expansion and improvement of the Kalman filter algorithm itself, mostly when applied to a non-linear system

Research into applying the Kalman filter to a non-linear system is active as the Kalman filter
was originally developed for a linear system. But most systems around us are non-linear.
Thus if the Kalman filter ca only be applied to a linear system, it's applications would be
very limited and it's value would not be acknowledged for a long time.

To apply the Kalman filter to non-linear systems, the extended Kalman filter (EKF) was developed. 
The EKF has been used successfully for 50 years in many different scenarios.
But the main shortcoming is that the EKF can diverge -- and most active research revolves around 
handling this issue. 

## Linearized Kalman Filter

Same algorithm as the linear Kalman filter, but the system model differs.

The system model of a linear Kalman filter is linear from the start.
The system model in a linearized Kalman filter is linearized from it's original non-linear form.

When applying a linearized Kalman filter to a real system, one must be careful about the
range of operation of the system. A linearized model is close to the real system
only around the point where it was linearized. Outside this range, the linearized model cannot be trusted;
it's performance would either degrade or even diverge. 

## Extended Kalman Filter

A representative algorithm for a Kalman filter expanded and applied to a non-linear system.
It has been proved to be reliable for decades through theory and practice.

The system model for EKF is non-linear:
```
x_{k+1} = f(x_k) + w_k
z_k = h(x_k) + v_k
```
The state variable `x_k` and coefficients are not separable.

The system model for linear Kalman filter:
```
x_{k+1} = A * x_k + w_k
z_k = H * x_k + v_k
```

In short, the linear matrix equation was change to a non-linear one:
```
A * x_k => f(x_k)
H * x_k => h(x_k)
```

### EKF Algorithm

```
0. Set initial values \hat{x}_0, P_0

1. Predict state and error covariance:
   \hat{x}^- = f(\hat{x}_{k-1})
   P_k^-     = A * P_{k-1} * A^T + Q
                                                  Measurements z_k
2. Compute Kalman Gain:                                /
   K_k = P_k^1 * H^T * (H * P_k^- * H^T + R)^{-1}     /
                                                     /
3. Compute the estimate                             v
   \hat{x}_k = \hat{x}_k^- + K_k * (z_k - h(\hat{x}^-))
                                       \
4. Compute the error covariance:        \
   P_k = P_k^- - K_k * H * P_k^-         \
                                          v
5. Goto Step 1.                       Estimate \hat{x}_k
```

The places where `A * x_k` and `H * x_k` are used in the linear algorithm model
are replaced by equations `f(\hat{x}_{k-1})` and `h(\hat{x}^-)` describing the non-linear model. 

However, the EKF algorithm still needs the system matrices `A` and `H` to
perform other computations. They need to be derived using the non-linear system model. 

In a linearized Kalman filter, `A` and `H` are already obtained through linearization
of the system model.

In the EKF, `A` and `H`  are obtained by linearizing the non-linear system model
about the previous estimate. 
```
A = \delta f |
    -------- |
    \delta x |x=\hat{x}_k 
    
H = \delta h |
    -------- |
    \delta x |x=\hat{x}_k^-     
```

The EKF uses `\hat{x}_k` as a reference point for linearization.
This gets a linear model based on the previous estimate which is assumed to be the closes
value to the actual state of the system. Thus the EKF is useful when the reference point for
linearization cannot be determined in advance. 

If the reference point for linearization can be determined in advance
(e.g. a satellite has a constant orbit, launch vehicle with a planned trajectory), 
then a linearized Kalman filter can be used. 

