# Unscented Kalman Filter

## Why was it developed?

The unscented Kalman Filter (UKF) is a popular substitute for the extended Kalman filter (EKF).
It's name may or may not refer to the fact that the EKF stinks, while the UKF doesn't stink.

The EKF solves the issue of non-linear system models with an expansion of the linear algorithm.
It takes the Jacobian of the non-linear system model and uses Euler integration to discretize it into matrix form.
While one generally gets good results, the downside to is that such a linear model could diverge.

The UKF eliminates the linearization process, thus the user does not need to worry about divergence. 
The downside to the UKF is that it more complex to understand and implement than the EKF.
However, it is still based on the core principles of predicting state variables based on the (nonlinear) system model,
then calibrating the prediction with measurements to get the final estimate. 

## Overview of Unscented Transformation

The UKF is based on unscented transformation. 
Consider a state variable `x` that follows a normal distribution with mean `xm` and covariance `Px`.
```
x ~ N(xm, Px)
```
We want to find the resultant mean and covariance when `x` is transformed by an arbitrary non-linear function `f(.)`.
There is no closed form solution for this, so we have to do numerical analysis and obtain an approximate solution.

The simplest and clearest approximate solution is to use the Monte Carlo Simulation,
which applies enough number of samples of `x ~ N(xm, Px)` into `f(x)` and calculates the resultant estimate
of mean and covariance directly. However, this method requires heavy computational power and is not suitable
for real-time use cases.

The Unscented Transformation is a more lightweight method that also randomly picks samples,
but uses weightings for each sample in order to reduce the number of samples needed. 

## Unscented Transformation Algorithm

The unscented transformation does not compute the mean and covariance of `f(x)` directly.
Instead, it estimates the mean and covariance from the transformed result of some representative values
(its sigma points).

1. Define the sigma points (`S{i}`) for `x`:
```
S{1} = xm

S{i+1} = xm + u{i}
  where i = {1,2,...,n}
  
S{i+n+1} = xm - u{i}
  where i = {1,2,...,n}
```
where `ui` is a row vector from matrix `U` and `K` is an arbitrary constant.
```
U'U = (n + K)Px
```
These sigma points `S{i}` of the unscented transformation correspond to the samples in the Monte Carlo simulation.

2. Define the weights (`W{i}`) for `x`:
```
         K
W{1} = -----
       n + K
     
            1
W{i+1} = -------- , where i = {1,2,...,n}
         2(n + K)   
         
W{i+n+1} =    1 
           -------- , where i = {1,2,...,n}
           2(n + K)          
```
These weights `W{i}` are the constants determining the weighting of each sigma point when computing mean and variance.

3. Compute the mean and covariance of `y = f(x)` -- which is the ultimate goal of the unscented transformation. 
```
     2n+1
ym = ∑    W{i} * f(S{i})
     i=1
     
     2n+1
Py = ∑    W{i} * [f(S{i}) - ym][f(S{i}) - ym]'
     i=1    
```
The sigma points and weights satisfy these characteristics. 
Notice that 
```
     2n+1
Py = ∑    W{i} * [f(S{i}) - ym][f(S{i}) - ym]'
     i=1  
```
contains the original mean and covariance -- so even without an infinite number of samples,
the statistical characteristics of `x` (e.g. mean, covariance) can be expressed by only `2n+1` 
sigma points and weights. 

As an example, a 2D state variable `x = {x1, x2}` has `5 (= 2 x 2 + 1)` sigma points and weights. 
The original distribution of `x` can be represented by just these 5 points, 

## Usage

In summary, the UKF is a non-linear Kalman filter that operates by modeling the probability distribution of a 
given non-linear function, instead of modeling the function itself. It computes the mean and covariance
of the function using statistical techniques.

The UKF is thus a good substitude for an EKF when the linearized model of the Jacobian is
unstable or hard to obtain. It is also practical as it's algorithm is not excessively complicated
to implement. 

In general, it is preferable to use the EKF when it is easy to obtain the Jacobian of a non-linear
system in an analytical way. This saves on computational time. 

When it is hard to obtain the Jacobian analytically, or there is a change the EKF could diverge,
it is preferable to use the UKF. 

When one is uncertain about whether to use the EKF or UKF, comparing the results from
both filters is the best way to choose between them.















