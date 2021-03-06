# Attitude Reference System

## Introduction

Suppose when measuring the horizontal attitude of a helicopter, we have measurements from two
sensors available: a gyroscope and an accelerometer.

The attitude estimated from a gyroscope are obtained by integration of the measurements of angular rate.
This estimate deviates along time due to error accumulation.
Due to this error accumulation, the accelerometer cannot be used to calibrate data.

When acceleration is constant, the attitude estimated from an accelerometer remains stable over time;
thus the acceleration can be used to calibrate data in this scenario.
But when acceleration or centrifugal/centripetal force changes,
the attitude estimated from an accelerometer grows with time. 

To overcome the shortcomings of the two sensors, they can be blended with a Kalman filter
to improve the estimates.

## Complementary filter 

The gyroscope and an accelerometer can also be combined with a complementary filter.

To apply a complementary filter to a sensor fusion system, the noise characteristics
of the two sensors must be complementary to each other. For example:
* A sensor that is sensitive to variation in a short period,
  but whose error slowly increases over time, is useful to measure short-term changes.
* A sensor that is not sensitive to variation in a short period,
  but that can represent changes over a long period well,
  is useful to measure long-term change. 

In the example system to measure attitude, the two sensors are complementary:
* The attitude measured by a gyroscope is accurate in the short-term,
  but due to error accumulation, it's inaccuracy increases proportional to time,
  This is the equivalent to  having a low-frequency noise mixed into the signal.
* The attitude measured by an accelerometer does not accumulate error over time.
  When noise is added to the acceleration, the error does get large,
  which is equivalent to having high-frequency noise mixed into the signal.

To design a complementary system to measure attitude:
```       
                                                                             -----
                                                                       (+)  | K_I |
                                                             ----- o <----- | --- | <---------------------
                                                            |      ^        |  s  |                       |
                                                            |      |         -----                        |
                                                            |      | (+)                                  |
                                                            |      |         -----                        |               
                                                            |       ------- | K_P | <---------------------|                                                            |                -----                        |
                                                            |                -----                        | 
                                                            |                           error in attitude | 
                                                            |                      between accel and gyro |              
                                                            | (-)                 -----                   |
 ---------------          ------------------          (+)   v                    |  1  |            (+)   |
|      Gyro     | -----> | Body -> Inertial | ------------> o -----------------> | --- | ---------------> o
 ---------------          ------------------   change in       change in final   |  s  |   final          | 
                                               roll angle      attitude angle     -----    attitude       | 
                                               from gyro       (dot_??m)                    angle (??m)     |
                                               (dot_??g)                                                   |
                                                                                                      (-) | 
                                                                                                          |
 ---------------          --------------------   roll angle from accelerometer (??a)                       |
| Accelerometer | -----> | Calculate attitude | ---------------------------------------------------------- 
 ---------------          --------------------
                          
```

The system works as follows:
* Use measurements from the accelerometer to compute the roll (`??a`) and the pitch (`??a`) angles.
* Find the error between the attitude estimated by the accelerometer, and the attitude estimated
  by the gyroscope. 
* Use the error between the two sensors as input to the proportional-integral `(K_P, K_I)` controller.
  The output of the `(K_P, K_I)` controller is used to calibrate the angular rate measured by the gyroscope.
  In other words, 
```


                                    Error between attitude from
                                    accelerometer and gyroscope
                                               |
                                               |
                                               V 
 -----------------------   subtract    -----------------------
| Angular rate measured | ----------> | Proportional-Integral | transforms attitute error
| by gyroscope          |             | Controller            | into the value used to calibrate
 -----------------------               -----------------------  angular rate
          |
          | integrate
          |
          V 
       Final attitude angle ??m
```

The algorithm of the complementary system has a feedback structure.

But the complementary filter itself is not to be a feedback loop - 
it is supposed to be an open-loop that does the following:
1. Filter noises with low-pass and high-pass filters
2. Sum the results of filtering the noises. 

## Checking that the algorithm is for a complementary filter

First define the error `e` between the attitude angles from the accelerometer and gyro to be:
```
e = ??m - ??a
```

The attitude angle `??m` from the gyro is computed by integration in this manner:
```
      1    .
??m = --- * ??m
      s 
      
      1    [ .        [       KI  ] ]
   = --- * [ ??g - e * [ KP + ---- ] ] 
      s    [          [        s  ] ]
      
      1    [ .                 [       KI  ] ]
   = --- * [ ??g - (??m - ??a)  * [ KP + ---- ] ] 
      s    [                   [        s  ] ]    
```

Multiplying `s * s` to both sides and rearranging, we get:
```
 2         2    1  [ .              [       KI  ] ]
s  * ??m = s  * --- [ ??g - (??m - ??a) [ KP + ---- ] ]
                s  [                [        s  ] ]
                
            [ .              [       KI  ] ]   
        = s [ ??g - (??m - ??a) [ KP + ---- ] ]
            [                [        s  ] ]
             
            . 
        = s ??g - (??m - ??a) * (s KP + KI)
        
            .
        = s ??g - (s KP + KI) ??m + (s KP + KI) ??a        
```
Rearranging, we get
```
[  2             ]        .
[ s  + s KP + KI ] ??m = s ??g + (s KP + KI) ??a
```
which is equivalent to
```
           s        .          s
??m = -------------- ??g + -------------- ??a
      2                   2
     s  + s KP + KI      s  + s KP + KI
      
            2
           s        [  1  .  ]     s KP + KI
   = -------------- [ --- ??g ] + -------------- ??a
      2             [  s     ]    2
     s  + s KP + KI              s  + s KP + KI
    
```

We can define a low-pass filter as
```
         s KP + KI  
G(s) = --------------
        2
       s  + s KP + KI  
```
and then `1 - G(s)` becomes a high-pass filer of the form:
```
                 s KP + KI
1 - G(s) = 1 - --------------
                2
               s  + s KP + KI  
            
           [  2            ]
           [ s + s KP + KI ] - [ s KP + KI]  
         = --------------------------------  
                    2
                   s  + s KP + KI    
                 
                 2  
                s   
         = --------------         
            2
           s  + s KP + KI   
```
Thus, we see that the final attitude from the system is:
```
                    [  1  .  ]
??m = [ 1 - G(s) ] * [ --- ??g ] + G(s) ??a
                    [  s     ]
                    
                        [  1  .  ]
   = high-pass-filter * [ --- ??g ] + low-pass-filter * ??a 
                        [  s     ]                 
``` 

where
```
 1  .  
--- ??g
 s
```
is the attitude obtained by integrating the angular rate measured from the gyroscope.

For the equation for final attitude from the system, we see that the algorithm
is a typical complementary filter (even if it looks like a feedback system at first glance).
