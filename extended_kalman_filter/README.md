# Extended Kalman Filter (EKF)

## Introduction

After 50 years of the Kalman Filter's introduction, most new papers published about it fall into these categories:
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


