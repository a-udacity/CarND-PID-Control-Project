# Term 2 PID Control Project

---

A PID controller project was implemented in C++ based on the Python source code written for the class quizzes.
Minor modifications to the `main.cpp` and `PID.h` file were made.
Previous value of CTE was stored for further use (`double prev_cte`).
In the following, the function implementation and PID parameter tuning method is discussed briefly.
A demonstration of the PID controller in action can be viewed [here](./demo.mp4).??

## Function Implementation

### PID::UpdateError

The PID errors were updated using the following code. As noted earlier, the previous CTE is stored for computing the D-error in subsequent time steps.

```
  p_error = cte;
  i_error = i_error + cte;
  d_error = cte - prev_cte;
  prev_cte = cte;
```

### PID::TotalError

This function evaluates the actual steering value and returns it to `main.cpp`.
The proportional, integral and derivative error terms are combined into the total error. It is noted that the final return value is restricted to lie in the range [-1,1] are required by the simulator.
The last two `if` statements threshold values to these ranges.

```
  double steer = -Kp*p_error-Kd*d_error-Ki*i_error;
  // ensure that steering value is within the allowed range.
  if(steer<-1.0) steer=-1.0;
  if(steer>1.0) steer=1.0;
  return steer;
```

```
pid.UpdateError(cte);
steer_value = pid.TotalError();
```
## Parameter Tuning

The PID parameters were chosen using manual tuning. The influcence of each parameter on the behavior was understood from a video on youtube (https://www.youtube.com/watch?v=4Y7zG48uHRo). The following steps were used:

1. Since steering angle and CTE are of order of 1.0, quarter (0.25) value was used as a starting point for Kp.
2. Gradually Kd was reduced to remove oscillations - a twiddle-inspired of manual tuning was used.
3. Finally, Ki was tuned, where the order of mangnitude was reduced upon each iteration till satisfactory beahvior was observed.

The final tune parameters are: `Kp=0.287`, `Kd=0.00072`, and `Ki=5.675`.