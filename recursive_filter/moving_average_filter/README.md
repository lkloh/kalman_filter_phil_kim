# Moving average filter

## Why was it developed?

With the average filter in Chapter 1, we see that noise can be removed by averaging.

Sometimes the quantity to be measured changes with time.
Simple averaging is not ok as all the dynamics of change are removed.

Moving averages can help remove noise and show the dynamic pattern of the system.

## What is a moving average?

Moving average is an average of a certain number of recent measurements.
The oldest data is discarded once a new data point is recorded - 
thats how the number of data points to get the average is maintained.

Moving average of `n` data points is expressed as 
```
\bar{x}_k = (x_{k-n+1} + x_{k-n+2} + ... + x_k) / n
```
The recursive form of the moving average filter is
```
\bar{x}_k = \bar_{x}_{k-1} + (x_k - x_{k-n}) / n
```

