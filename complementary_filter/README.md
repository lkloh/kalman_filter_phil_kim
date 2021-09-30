# Complementary Filter

## Introduction

The Kalman filter is often used in sensor fusion to blend various sensors. 
But it is not the only choice of filter available; a complementary filter can be used as a substitute.
A complementary filter is similar and easier to design than the extended Kalman filter,
and divergence will not occur. 
However, in order to use a complementary filter for sensor fusion, 
the various sensors must have complementary frequency characteristics. 

## A complementary filter system

A complementary filter can be used in sensor fusion only when the frequency characteristics of
the sensors are complementary to one another. 

For example, if one of the sensors is sensitive to the low-frequency component of the
physical quantity of interest, the other sensor should be good at detecting the high-frequency component. 

The follow system measures the same physical quantity with two different sensors:
```
                           -------
x(t) + noise_1(t) ------> | G1(s) | ------
                           -------        |
                                          v
                                           ---------> z(t) ≈ x(t)
                                          ^
                           -------        |
x(t) + noise_2(t) ------> | G1(s) | ------
                           -------
```
Where the variables are:
* `x(t)` the true value of the physical quantity of interest
* `noise_1(t)` is the noise from the first sensor
* `noise_2(t)` is the noise from the second sensor 
* `G1(s)` is the transfer function of the filter processing the measurements from the first sensor
* `G2(s)` is the transfer function of the filter processing the measurements from the second sensor
* `z(t)` is the final result of the measurement

The goal is to find the true original value `x(t)` of the physical quantity of interest by 
blending the result of filtered measurements from each sensor. 
To do so, the filters `G1(s)` and `G2(s)` must be designed according to the noise character of the measurement.
The measurement noise of each signal should be complementary to each other for this blending to be successful.

## Designing a complementary filter

A complementary filter has the structure with `G1(s)` and `G2(s)`:

```
G1(s) = 1 - G(s)
G2(s) G(s)
```

where `G(s)` can be a high-pass or low-pass filter. Applying these filters to the system, we get 
the following complementary filter:

```
                           ----------
x(t) + noise_1(t) ------> | 1 - G(s) | ------
                           ----------        |
                                             v +
                                              ---------> z(t) ≈ x(t)
                                             ^ +
                           ----------        |
x(t) + noise_2(t) ------> |   G(s)   | ------
                           ----------
```

The final result of the complementary filter can be computed by:
```
Z(s) = transfer1 * input + transfer2 * input

     = [1-G(s)]*[X(s)+noise1(s)] + G(s)*[X(s)+noise2(s)] 
      
     = [1-G(s)]*X(s) + [1-G(s)]*noise1(s) + G(s)*X(s) + G(s)*noise2(s)
     
     = X(s) - G(s)*X(s) + [1-G(s)]*noise1(s) + G(s)*X(s) + G(s)*noise2(s)
     
     = X(s) + [1-G(s)]*noise1(s) +  G(s)*noise2(s)
```

The true value of the physical quantity to be estimated, `X(s)`,
is not distorted by the filter `G(s)`. 

The signals that are affected by the complementary filter are the measurement
noises `noise1(s)` and `noise2(s)` of each sensor. 
If both measurement noises are removed by careful design of `G(s)`,
the value of `X(s)` will be the only remaining signal in the final output `Z(s)`.

If `noise1(s)` is in the low-frequency band and `noise2(s)` is in the high frequency band,
and we make `G(s)` to be a low-pass filter, then `[1-G(s)]*noise1(s) = high-pass-filter * low-freq-input` 
and `G(s)*noise2(s) = low-pass-filter * high-freq-noise` will both become very small, so we have

```
Z(s) = X(s) + [1-G(s)]*noise1(s) +  G(s)*noise2(s)
     = X(s) + (high-pass-filter * low-freq-input) + (low-pass-filter * high-freq-noise)
     = X(s) + ε1 + ε2
     ≈ X(s)
```

Since the two measurement noises `noise1(s)` and `noise2(s)` from the two sensors are complementary,
we can design a way to blend the sensors. Additionally, there is no feedback loop,
so this system will not diverge.

## Designing a complementary filter ≥ 3 sensors

If there are 3 measurement sensors, the filter configuration should be:
* `G1(s)` = Transfer function of the filter for the 1st measurement sensor
* `G2(s)` = Transfer function of the filter for the 2nd measurement sensor
* `1 - G1(s) - G2(s)` = Transfer function of the filter for the 3rd measurement sensor

To configure such a complementary filter, the measurement noise of the 3rd sensor `noise3(s)`
should not be able to pass through `1 - G1(s) - G2(s)`. 
