## The system model used in the Kalman filter

The low-pass filter has been shown to be capable removing noise from position measurements.
It does not know anything about the signal being measured, and just applies some weights
to the new measurement and old estimate before summing them.
Since it knows nothing about the signal being measured, it cannot estimate
a quantity that has not been measured. 

The Kalman filter has been show to be capable of removing noise and
also estimating velocity from position measurements.
It is able to estimate a quantity that has not been measured at all based on the system model it is given.
Thus, the relationship between the signal that is measured and other
state variables in the system model is known, and the Kalman filter can use this
relationship to estimate a state variable that has not been measured.

For example, if the velocity of a train is 80m/s, it moves by about 80m in 1s by the 
formula `displacement = velocity * delta_time`. Even if there is noise in the measured
position of the train, or some error in the system model, the train should still be
approximately at 80m. It should not be at a wildly different location, such as 800m. 

Similarly, if the distance covered by a train in 1s is 80m, its velocity should
be about 80m/s, not something wildly different such as 800m/s. 

In this way, the Kalman filter estimates the state variable by using information
from the system model. This means it is very important that the system model used for
estimate models the actual system closely. If it is very different from the 
actual system, the estimate would also be very different from the true value.
Even worse, the Kalman filter algorithm could diverge.
