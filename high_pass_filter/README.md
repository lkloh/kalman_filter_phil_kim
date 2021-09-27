# High-pass filter

## Introduction

A high-pass filter blocks the low-frequency component and outputs the high-frequency component only. 
It is useful for separating the operational domain of frequency of a controller.

For example, when a measurement is input to the high-pass filter, the measurement is filtered out,
and only the noise is output. This ability is useful for dampening aircraft vibration.
When an airplane flies, it's noise oscillates from left to right due to the external wind,
causing discomfort for it's passengers. To prevent this oscillation, most most passenger airplanes
use an automated yaw damper. 

An automated yaw damper could mistakenly categorize the pilot's intentional movement of the stick as 
oscillation of the noise and prevent the pilot from changing the direction of the aircraft. 
This mistake happens if the yaw damper cannot distinguish between nose movement caused by
intentional pilot operation and nose movement caused by external wind. A high-pass filter could be useful
for this purpose - the period of stick movement by the pilot is typically much longer (low frequency) and the period
of nose oscillation by external wind (high frequency). 

By attaching a high-pass filter to the sensor measuring motion of the airplane nose, only the oscillation
caused by the wind (and not the pilot) is returned as an output, which is in turn input to the yaw damper. 
In this way the yaw damper will only operate when the nose of the airplane oscillations due to the wind. 

## Laplace transformation 

Frequency-based filters such as low-pass and high-pass filters are designed based on the Laplace transformation,
which transforms functions in a time-domain to functions in the frequency domain. 

The Laplace transformation of an arbitrary function in the time-domain `g(t)` is:
```
       ∞         -st
G(s) = ∫ g(t) * e   dt
       0
```
when `G(s)` is the Laplace transformation of `g(t)`, represented in the frequency-domain. 

## Laplace filter

A filter is represented in this fashion:
```
                             --------------------
     u(t)                   |        g(t)        |
input signal -------------> | time-domain filter | ------------> y(t)
                             --------------------
                             
                             -------------------------
     U(t)                   |           G(t)          |
input signal -------------> | frequency-domain filter | --------> Y(s)
                             -------------------------                        
```

And the filter, input, and output signals have this relationship:
```
output   Y(s)
------ = ---- = G(s)  ===>  Output = Y(s) = G(s) * U(s) = (Transfer function) * Input
input    U(s)
```
`G(s)` is also known as a transfer function. The output is the multiplication of the transfer function and input.
However, in real-world scenarios it is rare to be able to compute the output with the transfer function and input 
as most input signals cannot be analytically expressed.

## Determining whether a filter is high-pass or low-pass

A transfer function of the form
```
         a
G(s) = ----- , a > 0
       s + a
```
is the simplest form of a frequency-based filter. 

To determined whether this function is a low-pass or high-pass filter, we check how the result changes 
according to the frequency band applied. In other words, as `s` increases or decreases, what happens to `G(s)`?

First find the limit of `G(s)` as `s -> ∞`:
``` 
                   a       a 
 lim G(s) = lim  ----- = ----- = 0
s->∞        s->∞ s + a   ∞ + a
```
When the frequency of the input signal is high (`s -> ∞`), `G(s)` converges to 0. 
This means the output `Y(s)` of the filter also converges to 0, meaning the high-frequency signal cannot pass through the filter. 

Now find the limit of `G(s)` as `s -> 0`:
``` 
                   a       a 
 lim G(s) = lim  ----- = ----- = 1
s->0        s->0 s + a   0 + a
```
When the frequency of the input signal is low (`s -> 0`), then `G(s)` converges to 1.
The output `Y(s)` of the filter is identical to the input signal,
meaning the input signal passes through the filter unchanged. 
Thus, we see that `G(s)` is a low-pass filter. 

## Expressing frequency filters through the Laplace Transformation

The simplest low-pass filter in the frequency domain is 
```
         a
G(s) = ----- , a > 0
       s + a
```
while the simplest low-pass filter in the time-domain is 
```            
estimated_x{k+1} = alpha * estimated_x{k} + (1 - alpha) * meas_x{k}
```
where `alpha` is a constant in the range `0 < alpha < 1`.

To compare these filters, we transform `G(s)` into the time-domain with the inverse Laplace transformation.
```
          a/a         1         1
G(s) = --------- = ------- = -----------
       s/a + a/a   s/a + 1   tau * s + 1
```
where `tau = 1/a`. Next,
```
(tau*s + 1) * estimated_X(s) = meas_x(s)
```
Applying the inverse Laplace transformation, the equation is represented in time domain as:
```
      d 
tau * -- estimate_x(t) + estimate_x(t) = meas_x(t)
      dt
```
Since the above is a continuous function of time, we need to discretize it to compare it with the
discrete formula for low-pass filter in the time-domain, `estimate_x{k+1} = alpha * estimate_x{k} + (1 - alpha) * meas_x`.

To do so, we substitute differentiation with:
```
d                  estimate_x{k+1} - estimate_x{k}
-- estimate_x(t) = -------------------------------
dt                            delta{t}
```
where `delta{t}` is the sampling interval. We can then rewrite to
```
      d 
tau * -- estimate_x(t) + estimate_x(t) = meas_x(t)
      dt
      
          estimate_x{k+1} - estimate_x{k}
==> tau * ------------------------------- + estimate_x{k} = meas_x{k+1}
                     delta{t}

                           tau                            delta{t}
==> estimate_x{k+1} = -------------- * estimate_x{k} + -------------- * meas_x{k+1}
                      tau + delta{t}                   tau + delta{t}
```
Comparing this with the low-pass filter in the time-domain, `estimate_x{k+1} = alpha * estimate_x{k} + (1 - alpha) * meas_x`,
we see that `alpha` satisfies the relationship: 
```
              tau
alpha = --------------
        tau + delta{t}
```
`alpha` was first introduced as a weighting for previous estimate and measurement.
Now we see that to decide its value, the frequency character and requirements of the filter
must be included in consideration. 
