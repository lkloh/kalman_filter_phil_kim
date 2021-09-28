# High-pass filter

## Introduction

A high-pass filter is a filter that takes a signal as input, then removes the low-frequency component to output
only the high-frequency component. It is the opposite of a low-pass filter.

An example of a simple high-pass filter is: 
```
         s
G(s) = -----  
       s + a    
```

## Determining the frequency character of the filter

We confirm the frequency character of the filter by observing how it converges as `s` is increased and decreased.

When `s` is decreased, the transfer function `G(s)` approaches 0, 
meaning the input signal cannot pass when it has low frequency.
```
                   s       0
lim  G(s) = lim  ----- = ----- = 0
s->0        s->0 s + a   0 + a
```


When `s` is increased, the transfer function `G(s)` approaches 1,
meaning the input signal passes as is when the frequency is high.
```
                   s             s/s         1        1
lim  G(s) = lim  ----- = lim  --------- = ------- = ----- = 1
s->∞        s->∞ s + a   s->∞ s/s + a/s   1 + a/∞   1 + 0 
```

It is clear that `G(s)` is a high-pass fillter.

## Transforming a frequency-domain filter to a time-domain filter for implementation

```
         s        s/a        tau * s
G(s) = ----- = --------- = -----------
       s + a   s/a + a/a   tau * s + 1
```
where `tau = 1/a`.

Applying the definition of the transfer function, we have
```
       output   Y_freq(s)     tau * s
G(s) = ------ = --------- = -----------
        input   U_freq(s)   tau * s + 1
```
Rearranging, we get
```
(tau * s + 1) * Y_freq(s) = tau * s * U_freq(s)
```
Applying the inverse Laplace transformation to both sides, we get:
```
      dy(t)                du(t)
tau * ----- + y(t) = tau * -----
       dt                    dt
```
To implement this, we must express it in discrete form. 

Define
```
dy(t)   y{k+1} - y{k}
----- = -------------
 dt        delta{t}
```
and
```
du(t)   u{k+1} - u{k}
----- = -------------
 dt        delta{t}
```
then substitute to get
```
      y{k+1} - y{k}                u{k+1} - u{k}
tau * ------------- + y(t) = tau * -------------
        delta{t}                      delta{t}
```
Rearranging, we get
```
              tau                    tau
y{k+1} = -------------- y{k} + -------------- (u{k+1} - u{k})
         tau + delta{t}        tau * delta{t}
```
Compared to the low-pass filter, this requires the previous input `u{k}` in addition
to the current input `u{k+1}`.

## Relationship between high-pass and low-pass filters

The transfer function for the low-pass filter is given as
```
         a
G(s) = -----
       s + a
```
and the transfer function of the high-pass filter is given as
```
         s
G(s) = -----
       s + a
```
Summing, we get:
```
G_low_pass(s) + G_high_pass(s) 

  low_pass_output   high_pass_output
= --------------- + ----------------
     input               input
     
  total_output  
= ------------
     input

    a       s
= ----- + -----
  s + a   s + a

  a + s
= -----
  s + a
  
= 1  
```
Clearly the relationship between the low-pass and high-pass filters is complementary.

